import math
import config


def normalize_angle(angle):
    while angle > 180:
        angle -= 360
    while angle < -180:
        angle += 360
    return angle


def compute_yaw(detection):
    if detection.pose_R is None:
        return None

    print("pose_R:")
    print(detection.pose_R)
    R = detection.pose_R
    yaw = math.degrees(math.atan2(R[1, 0], R[0, 0]))

    return normalize_angle(yaw)


def compute_x_offset(detection):
    if detection.pose_t is None:
        return None

    return float(detection.pose_t[0][0])


def update(detections):
    for detection in detections:
        detection.yaw = compute_yaw(detection)
        detection.x_offset = compute_x_offset(detection)
