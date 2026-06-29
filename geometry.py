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

    # print("pose_R:")
    # print(detection.pose_R)
    R = detection.pose_R
    yaw = math.degrees(math.atan2(R[1, 0], R[0, 0]))

    return normalize_angle(yaw)


def compute_x_offset(detection):
    if detection.pose_t is None:
        return None

    return float(detection.pose_t[0][0])


def rotation_matrix_to_euler(R):
    roll = math.degrees(math.atan2(R[2, 1], R[2, 2]))
    pitch = math.degrees(
        math.atan2(-R[2, 0], math.sqrt(R[2, 1] * R[2, 1] + R[2, 2] * R[2, 2]))
    )
    yaw = math.degrees(math.atan2(R[1, 0], R[0, 0]))

    return roll, pitch, yaw


def update(detections):
    for detection in detections:
        detection.yaw = compute_yaw(detection)
        detection.x_offset = compute_x_offset(detection)

        roll, pitch, yaw = rotation_matrix_to_euler(detection.pose_R)
        print(f"Roll: {roll:6.2f}, Pitch: {pitch:6.2f}, Yaw: {yaw:6.2f}")
