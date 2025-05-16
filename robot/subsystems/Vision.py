
import cv2
import depthai as dai
import numpy as np
from scipy.spatial import distance as dist
from pathlib import Path
import blobconverter
import argparse
import json
from robot.util.Subsystem import Subsystem  # Assuming Subsystem is in this relative path

class CentroidTracker:
    # ... (CentroidTracker class remains the same as before)
    def __init__(self, max_disappeared=5):
        self.next_object_id = 0
        self.objects = dict()  # object_id -> centroid (spatial_x, spatial_z)
        self.disappeared = dict()  # object_id -> consecutive missing count
        self.max_disappeared = max_disappeared

    def register(self, centroid):
        self.objects[self.next_object_id] = centroid
        self.disappeared[self.next_object_id] = 0
        self.next_object_id += 1

    def deregister(self, object_id):
        del self.objects[object_id]
        del self.disappeared[object_id]

    def update(self, detections, frame_width, frame_height):
        # no detections → mark all existing as disappeared
        if len(detections) == 0:
            for oid in list(self.disappeared):
                self.disappeared[oid] += 1
                if self.disappeared[oid] > self.max_disappeared:
                    self.deregister(oid)
            return self.objects

        # Extract centroids from detections
        input_centroids = []
        for detection in detections:
            # Use the center of the bounding box and the spatial coordinates
            cx = int((detection.xmin + detection.xmax) / 2 * frame_width)
            cy = int((detection.ymin + detection.ymax) / 2 * frame_height)
            sx = detection.spatialCoordinates.x / 1000  # Convert mm to meters
            sz = detection.spatialCoordinates.z / 1000  # Convert mm to meters
            input_centroids.append((cx, cy, sx, sz)) # Store cx, cy, spatial x, spatial z

        # first frame or no existing tracks → register all
        if len(self.objects) == 0:
            for c in input_centroids:
                # Register with spatial x and z
                self.register((c[2], c[3]))
            return self.objects

        # otherwise match input centroids to existing ones
        object_ids = list(self.objects.keys())
        object_centroids_2d = [obj[:2] for obj in self.objects.values()] # Use only 2D for matching
        input_centroids_2d = [c[:2] for c in input_centroids]

        if not object_centroids_2d or not input_centroids_2d:
            return self.objects  # Avoid errors if lists are empty

        # Calculate distances based on 2D centroid (x, y)
        D = dist.cdist(np.array(object_centroids_2d), np.array(input_centroids_2d))

        rows = D.min(axis=1).argsort()
        cols = D.argmin(axis=1)[rows]

        used_rows, used_cols = set(), set()
        updated_objects = {}
        for r, c in zip(rows, cols):
            if r in used_rows or c in used_cols:
                continue
            oid = object_ids[r]
            # Update tracked object with the new spatial x and z
            self.objects[oid] = (input_centroids[c][2], input_centroids[c][3])
            self.disappeared[oid] = 0
            updated_objects[oid] = (input_centroids[c][2], input_centroids[c][3])
            used_rows.add(r)
            used_cols.add(c)

        # any unmatched existing → disappeared
        for r in set(range(len(object_centroids_2d))) - used_rows:
            oid = object_ids[r]
            self.disappeared[oid] += 1
            if self.disappeared[oid] > self.max_disappeared:
                self.deregister(oid)

        # any unmatched new centroids → register
        for c in set(range(len(input_centroids_2d))) - used_cols:
            # Register with spatial x and z
            self.register((input_centroids[c][2], input_centroids[c][3]))

        return self.objects

class Vision(Subsystem):
    """
    Vision subsystem for detecting objects using DepthAI and tracking them,
    returning tracked object IDs with their X and Z coordinates, consistent with
    the original Vision subsystem.
    """
    def __init__(self, configPath='best.json', model_name='best_openvino_2022.1_6shave.blob', output_depth=False):
        self.command = None
        
        self.config_path = Path(configPath)
        self.model_name = model_name
        self.pipeline = None
        self.device = None
        self.preview_queue = None
        self.detection_nn_queue = None
        self.depth_queue = None  # New depth queue
        self.nn_config = {}
        self.metadata = {}
        self.labels = {}
        self.W, self.H = 416, 416  # Default input size
        self.tracker = CentroidTracker(max_disappeared=5)
        self.frame_width = 0
        self.frame_height = 0
        self.output_depth = output_depth  # Flag to output depth (internal use)
        self.current_frame = None
        self.current_detections = []
        self.current_depth_frame = None
        self._load_config()
        self._create_pipeline()
        self._start_device()

    def _load_config(self):
        if not self.config_path.exists():
            raise ValueError(f"Config path {self.config_path} does not exist!")
        with self.config_path.open() as f:
            config = json.load(f)
        self.nn_config = config.get("nn_config", {})
        if "input_size" in self.nn_config:
            self.W, self.H = tuple(map(int, self.nn_config.get("input_size").split('x')))
        self.metadata = self.nn_config.get("NN_specific_metadata", {})
        self.labels = config.get("mappings", {}).get("labels", {})

    def _create_pipeline(self):
        self.pipeline = dai.Pipeline()

        # Define sources and outputs
        camRgb = self.pipeline.create(dai.node.ColorCamera)
        detectionNetwork = self.pipeline.create(dai.node.YoloSpatialDetectionNetwork)
        monoLeft = self.pipeline.create(dai.node.MonoCamera)
        monoRight = self.pipeline.create(dai.node.MonoCamera)
        stereo = self.pipeline.create(dai.node.StereoDepth)
        xoutRgb = self.pipeline.create(dai.node.XLinkOut)
        xoutNN = self.pipeline.create(dai.node.XLinkOut)

        xoutRgb.setStreamName("rgb")
        xoutNN.setStreamName("detections")

        # Output for depth frame (if needed internally)
        if self.output_depth:
            xoutDepth = self.pipeline.create(dai.node.XLinkOut)
            xoutDepth.setStreamName("depth")
            stereo.depth.link(xoutDepth.input)

        # Properties
        camRgb.setPreviewSize(self.W, self.H)
        camRgb.setResolution(dai.ColorCameraProperties.SensorResolution.THE_720_P)
        camRgb.setInterleaved(False)
        camRgb.setColorOrder(dai.ColorCameraProperties.ColorOrder.BGR)
        camRgb.setFps(40)

        monoLeft.setResolution(dai.MonoCameraProperties.SensorResolution.THE_400_P)
        monoLeft.setCamera("left")
        monoRight.setResolution(dai.MonoCameraProperties.SensorResolution.THE_400_P)
        monoLeft.out.link(stereo.left)
        monoRight.out.link(stereo.right)

        stereo.setDefaultProfilePreset(dai.node.StereoDepth.PresetMode.HIGH_DENSITY)
        stereo.setDepthAlign(dai.CameraBoardSocket.CAM_A)
        stereo.setOutputSize(monoLeft.getResolutionWidth(), monoLeft.getResolutionHeight())
        stereo.setSubpixel(True)

        # Network specific settings
        nnPath = self.model_name
        if not Path(nnPath).exists():
            print(f"No blob found at {nnPath}. Looking into DepthAI model zoo.")
            nnPath = str(blobconverter.from_zoo(self.model_name, shaves=6, zoo_type="depthai", use_cache=True))
        detectionNetwork.setBlobPath(nnPath)
        detectionNetwork.setConfidenceThreshold(self.metadata.get("confidence_threshold", 0.5))
        detectionNetwork.input.setBlocking(False)
        detectionNetwork.setBoundingBoxScaleFactor(0.5)
        detectionNetwork.setDepthLowerThreshold(100)
        detectionNetwork.setDepthUpperThreshold(5000)
        detectionNetwork.setNumClasses(self.metadata.get("classes", 80))
        detectionNetwork.setCoordinateSize(self.metadata.get("coordinates", 4))
        detectionNetwork.setAnchors(self.metadata.get("anchors", []))
        detectionNetwork.setAnchorMasks(self.metadata.get("anchor_masks", {}))
        detectionNetwork.setIouThreshold(self.metadata.get("iou_threshold", 0.5))
        detectionNetwork.setNumInferenceThreads(2)

        # Linking
        camRgb.preview.link(detectionNetwork.input)
        detectionNetwork.passthrough.link(xoutRgb.input)
        detectionNetwork.out.link(xoutNN.input)
        stereo.depth.link(detectionNetwork.inputDepth)

    def _start_device(self):
        try:
            self.device = dai.Device(self.pipeline)
            self.preview_queue = self.device.getOutputQueue(name="rgb", maxSize=4, blocking=False)
            self.detection_nn_queue = self.device.getOutputQueue(name="detections", maxSize=4, blocking=False)
            if self.output_depth:
                self.depth_queue = self.device.getOutputQueue(name="depth", maxSize=4, blocking=False)
            else:
                self.depth_queue = None
        except Exception as e:
            print(f"Error connecting to DepthAI device: {e}")
            self.pipeline = None
            self.device = None

    def process_frame(self, label_name="Red-Ring"):
        if self.pipeline is None or self.device is None:
            print("DepthAI device not initialized.")
            return {}

        in_preview = self.preview_queue.get()
        in_detections = self.detection_nn_queue.get()
        self.current_frame = in_preview.getCvFrame()
        
        self.current_detections = []

        for detection in in_detections.detections:
            label = self.labels[detection.label]
            conf = detection.confidence
            if label == label_name and conf > 0.9:
                self.current_detections.append(detection)
            else:
                pass


        if self.output_depth and self.depth_queue is not None:
            in_depth = self.depth_queue.get()
            self.current_depth_frame = in_depth.getCvFrame()
        else:
            self.current_depth_frame = None

        self.frame_height, self.frame_width = self.current_frame.shape[:2]
        tracked_objects = self.tracker.update(self.current_detections, self.frame_width, self.frame_height)

        # Prepare the output dictionary with tracked IDs and their X, Z coords
        tracked_coords = {}
        for obj_id, coords in tracked_objects.items():
            tracked_coords[obj_id] = coords  # (spatial_x, spatial_z)

        return tracked_coords

    def get_frame(self):
        return self.current_frame

    def get_detections(self):
        return self.current_detections

    def get_depth_frame(self):
        return self.current_depth_frame

    def stop(self):
        if self.device is not None:
            self.device.close()
        cv2.destroyAllWindows()
        print("DepthAI pipeline stopped.")

