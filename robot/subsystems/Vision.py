
import cv2
import depthai as dai
import numpy as np
from scipy.spatial import distance as dist
from typing import List, Optional, Dict, Tuple
from pathlib import Path
import blobconverter
import argparse
import json
from robot.util.Subsystem import Subsystem  # Assuming Subsystem is in this relative path

    
class VisionDetection:
    label: str
    confidence: float
    xmin: float
    ymin: float
    xmax: float
    ymax: float
    spatialCoordinates: Optional[dai.Point3f] = None


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
        self.frame_width = 0
        self.frame_height = 0
        self.output_depth = output_depth  # Flag to output depth (internal use)
        self.current_frame = None
        self.current_raw_detections = []
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

    def get_raw_detections(self) -> List[VisionDetection]:
        if self.pipeline is None or self.device is None:
            print("DepthAI device not initialized.")
            return []
        
        inDetections = self.detection_nn_queue.get()
        if inDetections is not None:
            detections = []
            for detection in inDetections.detections:
                label = self.labels[detection.label]
                det = VisionDetection(
                    label=label,
                    confidence=detection.confidence,
                    xmin=detection.xmin,
                    ymin=detection.ymin,
                    xmax=detection.xmax,
                    ymax=detection.ymax,
                    spatialCoordinates=detection.spatialCoordinates
                )
                detections.append(det)
            return detections
        return []


    # def process_detections(self, label_name="Red-Ring"):
    #     if self.pipeline is None or self.device is None:
    #         print("DepthAI device not initialized.")
    #         return {}

    #     inPreview = self.preview_queue.get()
    #     inDetections = self.detection_nn_queue.get()
    #     inDepth = self.depth_queue.get()

    #     self.frame = inPreview.getCvFrame()
    #     self.depthFrame = inDepth.getCvFrame()
        
    #     self.height = self.frame.shape[0]
    #     self.width = self.frame.shape[1]
        
    #     self.current_detections = []
        

    #     for detection in inDetections.detections:
    #         label = self.labels[detection.label]
    #         roiData = detection.boundingBoxMapping
    #         roi = roiData.roi
    #         roi = roi.denormalize(self.depthFrame.shape[1], self.depthFrame.shape[0])
    #         topLeft = roi.topLeft()
    #         bottomRight = roi.bottomRight()
    #         xmin = int(topLeft.x)
    #         ymin = int(topLeft.y)
    #         xmax = int(bottomRight.x)
    #         ymax = int(bottomRight.y)
    #         cv2.rectangle(self.depthFrame, (xmin, ymin), (xmax, ymax), (255,0,0), 1)

    #         # Denormalize bounding box
    #         x1 = int(detection.xmin * self.width)
    #         x2 = int(detection.xmax * self.width)
    #         y1 = int(detection.ymin * self.height)
    #         y2 = int(detection.ymax * self.height)


    #         cv2.putText(self.frame, str(label), (x1 + 10, y1 + 20), cv2.FONT_HERSHEY_TRIPLEX, 0.5, 255)
    #         cv2.putText(self.frame, "{:.2f}".format(detection.confidence*100), (x1 + 10, y1 + 35), cv2.FONT_HERSHEY_TRIPLEX, 0.5, 255)
    #         cv2.putText(self.frame, f"X: {int(detection.spatialCoordinates.x)} mm", (x1 + 10, y1 + 50), cv2.FONT_HERSHEY_TRIPLEX, 0.5, 255)
    #         cv2.putText(self.frame, f"Y: {int(detection.spatialCoordinates.y)} mm", (x1 + 10, y1 + 65), cv2.FONT_HERSHEY_TRIPLEX, 0.5, 255)
    #         cv2.putText(self.frame, f"Z: {int(detection.spatialCoordinates.z)} mm", (x1 + 10, y1 + 80), cv2.FONT_HERSHEY_TRIPLEX, 0.5, 255)

    #         cv2.rectangle(self.frame, (x1, y1), (x2, y2), (255,0,0), cv2.FONT_HERSHEY_SIMPLEX)

    #     cv2.putText(self.frame, "NN fps: {:.2f}".format(fps), (2, self.frame.shape[0] - 4), cv2.FONT_HERSHEY_TRIPLEX, 0.4, (255,0,0))
    #     cv2.imshow("rgb", self.frame)



    #     if self.output_depth and self.depth_queue is not None:
    #         in_depth = self.depth_queue.get()
    #         self.current_depth_frame = in_depth.getCvFrame()
    #     else:
    #         self.current_depth_frame = None

    #     self.frame_height, self.frame_width = self.current_frame.shape[:2]
    #     tracked_objects = self.tracker.update(self.current_detections, self.frame_width, self.frame_height)


    #     # Prepare the output dictionary with tracked IDs and their X, Z coords
    #     tracked_coords = {}
    #     for obj_id, coords in tracked_objects.items():
    #         tracked_coords[obj_id] = coords  # (spatial_x, spatial_z)

    #     return tracked_coords

    def get_frame(self):
        return self.current_frame

    def get_detections(self, label_filter: Optional[str] = None) -> List[VisionDetection]:
        detections = self.get_raw_detections()
        if label_filter:
            return [det for det in detections if det.label == label_filter]
        return detections
    


    def get_depth_frame(self):
        return self.current_depth_frame

    def stop(self):
        if self.device is not None:
            self.device.close()
        cv2.destroyAllWindows()
        print("DepthAI pipeline stopped.")

