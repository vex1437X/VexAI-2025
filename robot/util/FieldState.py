from enum import Enum
import numpy as np
from typing import List, Optional, Dict, Tuple
from dataclasses import dataclass
from collections import Counter

class FieldObjectType(Enum):
    RED = "Red-Ring"
    BLUE = "Blue-Ring"
    GOAL = "Mobile-Goal"


class FieldObject:
    def __init__(self, label: str, field_x: float, field_y: float, confidence: float, bbox_pixels: Tuple[int, int, int, int]):
        self.label = label
        self.field_x = field_x
        self.field_y = field_y
        self.confidence = confidence
        self.bbox_pixels = bbox_pixels


class ObjectTracker:
    def __init__(self, vision, camera_offset_x: float, camera_offset_y: float, camera_height: float, camera_pitch: float, matching_threshold: float = 0.1):
        self.vision = vision
        self.camera_offset_x = camera_offset_x
        self.camera_offset_y = camera_offset_y
        self.camera_height = camera_height
        self.camera_pitch = np.deg2rad(camera_pitch)
        self.tracked_rings_field: List[FieldObject] = []
        self.tracked_mobile_goals_field: List[FieldObject] = []
        self.matching_threshold = matching_threshold  # Max distance to consider the same ring




    def transform_camera_to_field(self, camera_x: float, camera_y: float, camera_z: float, robot_x_field: float, robot_y_field: float, robot_heading_field: float) -> Tuple[float, float]:
        """Transforms a point from camera coordinates to field XY coordinates."""
        robot_x_relative_cam = camera_x - self.camera_offset_x
        robot_y_relative_cam = camera_y - self.camera_offset_y
        robot_z_relative_cam = camera_z - self.camera_height

        pitched_y = robot_y_relative_cam * np.cos(self.camera_pitch) + robot_z_relative_cam * np.sin(self.camera_pitch)
        pitched_z = -robot_y_relative_cam * np.sin(self.camera_pitch) + robot_z_relative_cam * np.cos(self.camera_pitch)
        pitched_x = robot_x_relative_cam

        cos_heading = np.cos(robot_heading_field)
        sin_heading = np.sin(robot_heading_field)

        field_x_relative_robot = pitched_x * cos_heading - pitched_y * sin_heading
        field_y_relative_robot = pitched_x * sin_heading + pitched_y * cos_heading

        field_x = robot_x_field + field_x_relative_robot
        field_y = robot_y_field + field_y_relative_robot

        return field_x, field_y
    
    @staticmethod
    def _check_overlap(bbox1: Tuple[int, int, int, int], bbox2: Tuple[int, int, int, int]) -> bool:
        """Checks if two bounding boxes overlap."""
        x1_1, y1_1, x2_1, y2_1 = bbox1
        x1_2, y1_2, x2_2, y2_2 = bbox2
        if x1_1 >= x2_2 or x1_2 >= x2_1: 
            return False
        if y1_1 >= y2_2 or y1_2 >= y2_1: 
            return False
        return True
    
    def update_tracked_objects(self, robot_x_field: float, robot_y_field: float, robot_heading_field: float):
        """Updates the tracked positions of rings and mobile goals in field coordinates."""
        self.vision.process_frame()
        current_detections = self.vision.get_detections()
        newly_detected_loose_rings: List[FieldObject] = []
        current_mobile_goals: List[FieldObject] = []

        if self.vision.current_frame is not None:
            frame_height, frame_width = self.vision.current_frame.shape[:2]
        else:
            return  # Cannot proceed without frame dimensions
        
        # Get mobile goal detections for overlap checking later
        mobile_goal_detections = [det for det in current_detections if "Mobile-Goal" in det.label]

        for detection in current_detections:
            if detection.spatialCoordinates:
                field_x, field_y = self.transform_camera_to_field(
                    detection.spatialCoordinates.x / 1000, # Convert mm to meters
                    detection.spatialCoordinates.y / 1000, # Convert mm to meters
                    detection.spatialCoordinates.z / 1000, # Convert mm to meters
                    robot_x_field,
                    robot_y_field,
                    robot_heading_field
                )
                bbox_pixels = (
                    int(detection.xmin * frame_width),
                    int(detection.ymin * frame_height),
                    int(detection.xmax * frame_width),
                    int(detection.ymax * frame_height),
                )
                field_object = FieldObject(detection.label, field_x, field_y, detection.confidence, bbox_pixels)

                if "Ring" in detection.label:
                    is_overlapping = False
                    for goal_det in mobile_goal_detections:
                        goal_bbox_pixels = (
                            int(goal_det.xmin * frame_width), int(goal_det.ymin * frame_height),
                            int(goal_det.xmax * frame_width), int(goal_det.ymax * frame_height)
                        )
                        if self._check_overlap(bbox_pixels, goal_bbox_pixels):
                            is_overlapping = True
                            break
                        if not is_overlapping:
                            newly_detected_loose_rings.append(field_object)
                elif "Mobile-Goal" in detection.label:
                    current_mobile_goals.append(field_object)

            #Update tracked rings
            updated_loose_rings: List[FieldObject] = []
            matched_indices = set()

            for new_ring in newly_detected_loose_rings:
                best_match_index = -1
                min_distance = float('inf')

            for i, tracked_ring in enumerate(self.tracked_loose_rings_field):
                distance = np.sqrt((new_ring.field_x - tracked_ring.field_x)**2 + (new_ring.field_y - tracked_ring.field_y)**2)
                if distance < self.matching_threshold and distance < min_distance:
                    min_distance = distance
                    best_match_index = i

            if best_match_index != -1 and best_match_index not in matched_indices:
                # Update existing tracked ring
                updated_loose_rings.append(new_ring)
                matched_indices.add(best_match_index)
            else:
                # Add as a new loose ring
                updated_loose_rings.append(new_ring)

            # Keep the previously tracked rings that were not matched in the current view
            for i, tracked_ring in enumerate(self.tracked_loose_rings_field):
                if i not in matched_indices:
                    updated_loose_rings.append(tracked_ring)

            self.tracked_loose_rings_field = updated_loose_rings
            self.tracked_mobile_goals_field = current_mobile_goals  # Update mobile goals every frame



    def get_tracked_loose_rings_field(self) -> List[FieldObject]:
        """Returns the list of tracked rings in field coordinates."""
        return self.tracked_rings_field

    def get_tracked_mobile_goals_field(self) -> List[FieldObject]:
        """Returns the list of tracked mobile goals in field coordinates."""
        return self.tracked_mobile_goals_field
    
