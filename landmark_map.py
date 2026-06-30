"""
LandmarkMap
    # Load JSON
    # Find landmark from any detected tag
    # Get landmark by ID
    # Get neighbors
    # Get next landmark
    # Compute desired heading
    # Compute distance
    # Path planning
"""

import json
import math


class LandmarkMap:
    def __init__(self, filename):
        with open(filename, "r") as file:
            self.data = json.load(file)

    # Return landmark dictionary by landmark ID
    def get_landmark(self, landmark_id):
        for landmark in self.data["landmarks"]:
            if landmark["id"] == landmark_id:
                return landmark

        return None

    # Return the landmark and tag position corresponding to any detected AprilTag.
    def find_landmark(self, tag_id):
        for landmark in self.data["landmarks"]:
            for position, tag in landmark["tags"].items():
                if tag == tag_id:
                    return {"landmark": landmark, "position": position}

        return None

    # Return neighboring landmark IDs
    def get_neighbors(self, landmark_id):
        landmark = self.get_landmark(landmark_id)

        if landmark is None:
            return None

        neighbors = []

        row = landmark["row"]
        column = landmark["column"]

        for candidate in self.data["landmarks"]:
            if candidate["id"] == landmark_id:
                continue

            dr = abs(candidate["row"] - row)
            dc = abs(candidate["column"] - column)

            if dr + dc == 1:
                neighbors.append(candidate["id"])

        return neighbors

    # Compute desired heading from current landmark to target landmark.
    #
    # World Coordinate System:
    #
    #       North (0°)
    #           ^
    #           |
    # West      |      East
    # 270°      |      90°
    #           |
    #           v
    #      South (180°)
    #
    def get_heading(self, current_id, target_id):
        current = self.get_landmark(current_id)
        target = self.get_landmark(target_id)

        if current is None or target is None:
            return None

        dr = target["row"] - current["row"]
        dc = target["column"] - current["column"]

        if dr < 0 and dc == 0:
            return 0.0

        if dr == 0 and dc > 0:
            return 90.0

        if dr > 0 and dc == 0:
            return 180.0

        if dr == 0 and dc < 0:
            return 270.0

        return None

    # For now, if the target is a neighbor,
    # simply return it.
    def get_next_landmark(self, current_id, target_id):
        neighbors = self.get_neighbors(current_id)

        if target_id in neighbors:
            return target_id

        return None
