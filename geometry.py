import math
import config


def normalize_angle(angle):
    while angle > 180:
        angle -= 360
    while angle < -180:
        angle += 360
    return angle


def compute_heading(detection):
    if detection.pose_R is None:
        return None

    R = detection.pose_R

    heading = math.degrees(math.atan2(R[1, 0], R[0, 0]))

    return normalize_angle(heading)


def compute_lateral(detection):
    if detection.pose_t is None:
        return None

    return float(detection.pose_t[0][1])


def compute_distance(detection):
    if detection.pose_t is None:
        return None

    return float(detection.pose_t[0][2])


def update(detections):
    for detection in detections:
        detection.heading = compute_heading(detection)
        detection.lateral = compute_lateral(detection)
        detection.distance = compute_distance(detection)
