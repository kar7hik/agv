import cv2
import serial
import math
import json
import time

from picamera2 import Picamera2
from pupil_apriltags import Detector
import numpy as np

# Camera Parameters:
FRAME_WIDTH = 640
FRAME_HEIGHT = 480
FX = 615.0
FY = 615.0
CX = FRAME_WIDTH / 2.0
CY = FRAME_HEIGHT / 2.0
TAG_SIZE_M = 0.01
CAMERA_PARAMS = (FX, FY, CX, CY)
APRILTAG_FAMILY = "tag36h11"

WINDOW_NAME = "AGV Navigation System"
PRINT_PERIOD_S = 0.5

# Map Parameters
MAP_FILE = "./maps/workspace.json"
EXPECTED_LANDMARK_ID = 0
PATH_HEADING_DEG = 0.0

HELPER_OFFSETS_M = {
    "north_west": (-0.015, +0.015),
    "north": (0.000, +0.015),
    "north_east": (+0.015, +0.015),
    "west": (-0.015, 0.000),
    "center": (0.000, 0.000),
    "east": (+0.015, 0.000),
    "south_west": (-0.015, -0.015),
    "south": (0.000, -0.015),
    "south_east": (+0.015, -0.015),
}


# Drive Parameters
DRIVE_SPEED_MPS = 0.030


COLOR_OPTICAL_CENTER = (180, 180, 180)
COLOR_NORMAL_TAG = (0, 220, 0)
COLOR_UNEXPECTED_TAG = (0, 0, 255)
COLOR_CENTER_LINE = (255, 180, 0)
COLOR_HEADING_ARROW = (0, 255, 255)
COLOR_TEXT = (255, 255, 255)


# Serial Parameters
SERIAL_PORT = "/dev/ttyUSB0"
SERIAL_BAUD = 115200

READ_TIMEOUT_S = 0.1
ACK_TIMEOUT_S = 0.5
CAL_TIMEOUT_S = 20.0




def open_serial():
    ser = serial.Serial(SERIAL_PORT, SERIAL_BAUD, timeout=READ_TIMEOUT_S)
    time.sleep(2.0)
    ser.reset_input_buffer()
    ser.reset_output_buffer()
    return ser


def read_line(ser):
    line = ser.readline().decode("utf-8", errors="ignore").strip()
    if line == "":
        return None
    return line


def clear_serial_buffer(ser):
    while ser.in_waiting > 0:
        line = read_line(ser)
        if line is not None:
            print(f"RX: {line}")


def send_line(ser, line):
    print(f"TX: {line}")
    ser.write((line + "\n").encode("utf-8"))
    ser.flush()


def wait_for_ack(ser, timeout=ACK_TIMEOUT_S):
    deadline = time.monotonic() + timeout

    while time.monotonic() < deadline:
        line = read_line(ser)
        if line is None:
            continue

        print(f"RX: {line}")

        if line.startswith("ACK"):
            return True

        if line.startswith("ERR") or line.startswith("FAULT"):
            return False

    print("RX: ACK timeout")
    return False


def send_command_wait_ack(ser, command, timeout=ACK_TIMEOUT_S):
    send_line(ser, command)
    return wait_for_ack(ser, timeout)


# Robot Commands
def ping(ser):
    return send_command_wait_ack(ser, "PING")


def motor_on(ser):
    return send_command_wait_ack(ser, "MOTOR_ON")


def motor_off(ser):
    return send_command_wait_ack(ser, "MOTOR_OFF")


def stop(ser):
    return send_command_wait_ack(ser, "STOP")


def cal_imu(ser):
    return send_command_wait_ack(ser, "CAL_IMU", timeout=CAL_TIMEOUT_S)


def zero_imu(ser):
    return send_command_wait_ack(ser, "ZERO_IMU")


def sync_heading(ser, heading_deg):
    command = f"SYNC {heading_deg:.2f}"
    return send_command_wait_ack(ser, command)


def drive(ser, velocity_mps, path_heading_deg, lateral_error_m):
    command = f"DRIVE {velocity_mps:.4f} {path_heading_deg:.4f} {lateral_error_m:.4f}"
    return send_command_wait_ack(ser, command)


def status(ser, timeout=1.0):
    send_line(ser, "STATUS")

    deadline = time.monotonic() + timeout

    while time.monotonic() < deadline:
        line = read_line(ser)
        if line is None:
            continue

        print(f"RX: {line}")

        if line.startswith("STATUS"):
            return line

    print("RX: STATUS timeout")
    return None


# MAP:
def load_map(filename):
    with open(filename, "r", encoding="utf-8") as file:
        map_data = json.load(file)

    if "grid" not in map_data:
        raise ValueError("Map does not contain 'grid'.")

    if "landmarks" not in map_data:
        raise ValueError("Map does not contain 'landmarks'.")

    return map_data


def build_map_lookup(map_data):
    if "landmarks" not in map_data:
        raise ValueError("Map does not contain landmarks.")

    landmark_lookup = {}
    tag_lookup = {}

    for landmark in map_data["landmarks"]:
        landmark_id = landmark["id"]

        if landmark_id in landmark_lookup:
            raise ValueError(f"Duplicate landmark ID: {landmark_id}")

        landmark_lookup[landmark_id] = landmark
        tags = landmark.get("tags")

        if not isinstance(tags, dict):
            raise ValueError(f"Landmark {landmark_id} does not contain tags.")

        for position, tag_id_value in tags.items():
            tag_id = int(tag_id_value)

            if tag_id in tag_lookup:
                previous = tag_lookup[tag_id]

                raise ValueError(
                    f"Tag ID: {tag_id} is used by both "
                    f"landmark {previous['landmark_id']} and landmark {landmark_id}."
                )

            tag_lookup[tag_id] = {
                "landmark_id": landmark_id,
                "position": position,
            }

    return landmark_lookup, tag_lookup


# Camera and Apriltag Detector:
def open_camera():
    camera = Picamera2()
    configuration = camera.create_preview_configuration(
        main={
            "format": "RGB888",
            "size": (
                FRAME_WIDTH,
                FRAME_HEIGHT,
            ),
        }
    )

    camera.configure(configuration)
    camera.start()

    # Allowing exposure and white balance to settle.
    time.sleep(1.0)
    return camera


def get_frame(cam):
    return cam.capture_array()


def create_apriltag_detector():
    return Detector(
        families=APRILTAG_FAMILY,
        nthreads=4,
        quad_decimate=1.0,
        quad_sigma=0.0,
        refine_edges=True,
    )


def detect_apriltags(frame, detector):
    gray_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

    detections = detector.detect(
        gray_frame,
        estimate_tag_pose=True,
        camera_params=CAMERA_PARAMS,
        tag_size=TAG_SIZE_M,
    )

    return detections


# Helper functions:
def normalize_angle_deg(angle_deg):
    while angle_deg > 180.0:
        angle_deg -= 360.0

    while angle_deg < -180.0:
        angle_deg += 360.0

    return angle_deg


def compute_tag_heading(detection):
    if detection.pose_R is None:
        return None
    return normalize_angle_deg(
        math.degrees(math.atan2(detection.pose_R[1, 0], detection.pose_R[0, 0]))
    )


def compute_tag_lateral(detection):
    return float(detection.pose_t[0][0]) if detection.pose_t is not None else None


def enrich_detections(detections):
    for detection in detections:
        detection.heading = compute_tag_heading(detection)
        detection.lateral = compute_tag_lateral(detection)


def optical_center_dist_squared(detection):
    delta_x_px = float(detection.center[0]) - CX
    delta_y_px = float(detection.center[1]) - CY

    return delta_x_px * delta_x_px + delta_y_px * delta_y_px


def is_usable_detection(detection, tag_lookup):
    tag_id = int(detection.tag_id)
    tag_info = tag_lookup.get(tag_id)

    if tag_info is None:
        return False

    if detection.hamming != 0:
        return False

    if detection.pose_t is None:
        return False

    if detection.pose_R is None:
        return False

    return True


def compute_landmark_center_pose(detection, tag_lookup):
    tag_id = int(detection.tag_id)
    tag_info = tag_lookup.get(tag_id)

    position = tag_info["position"]
    helper_x_m, helper_y_m = HELPER_OFFSETS_M[position]

    detected_tag_pose = np.asarray(detection.pose_t, dtype=np.float64).reshape(3)
    tag_rotation = np.asarray(detection.pose_R, dtype=np.float64).reshape(3, 3)

    helper_offset = np.array([helper_x_m, helper_y_m, 0.0], dtype=np.float64)
    helper_offset_in_camera = tag_rotation @ helper_offset
    landmark_center_pose = detected_tag_pose - helper_offset_in_camera

    return landmark_center_pose


def compute_landmark_center_lateral(detection, tag_lookup):
    center_pose = compute_landmark_center_pose(detection, tag_lookup)
    return float(center_pose[0])


def select_best_detection(detections, tag_lookup):
    usable_detections = []

    for detection in detections:
        if is_usable_detection(detection, tag_lookup):
            usable_detections.append(detection)

    if len(usable_detections) == 0:
        return None

    return min(
        usable_detections,
        key=lambda detection: (
            optical_center_dist_squared(detection),
            -float(detection.decision_margin),
        ),
    )


def show_frame(frame):
    cv2.imshow(WINDOW_NAME, frame)


def draw_frame(frame, detections, status_lines=None, highlight_ids=None):
    if highlight_ids is None:
        highlight_ids = set()

    height, width = frame.shape[:2]
    optical_center_x = int(round(CX))
    optical_center_y = int(round(CY))
    optical_center = (optical_center_x, optical_center_y)

    cv2.line(
        frame,
        (0, optical_center_y),
        (width - 1, optical_center_y),
        COLOR_OPTICAL_CENTER,
        thickness=1,
        lineType=cv2.LINE_AA,
    )

    cv2.line(
        frame,
        (optical_center_x, 0),
        (optical_center_x, height - 1),
        COLOR_OPTICAL_CENTER,
        thickness=1,
        lineType=cv2.LINE_AA,
    )

    cv2.circle(
        frame,
        optical_center,
        radius=5,
        color=COLOR_OPTICAL_CENTER,
        thickness=-1,
        lineType=cv2.LINE_AA,
    )

    for detection in detections:
        tag_id = int(detection.tag_id)

        corners = np.rint(detection.corners).astype(np.int32)
        center_x = int(round(detection.center[0]))
        center_y = int(round(detection.center[1]))
        center = (center_x, center_y)

        is_unexpected = tag_id in highlight_ids

        if is_unexpected:
            tag_color = COLOR_UNEXPECTED_TAG
            boundary_thickness = 3

        else:
            tag_color = COLOR_NORMAL_TAG
            boundary_thickness = 2

        cv2.polylines(
            frame,
            [corners],
            isClosed=True,
            color=tag_color,
            thickness=boundary_thickness,
            lineType=cv2.LINE_AA,
        )

        cv2.circle(
            frame,
            center,
            radius=5,
            color=tag_color,
            thickness=-1,
            lineType=cv2.LINE_AA,
        )

        cv2.line(
            frame,
            optical_center,
            center,
            COLOR_CENTER_LINE,
            thickness=1,
            lineType=cv2.LINE_AA,
        )

        tag_left = int(np.min(corners[:, 0]))
        tag_top = int(np.min(corners[:, 1]))
        tag_bottom = int(np.max(corners[:, 1]))

        text_x = max(5, tag_left)

        label_text_y = max(18, tag_top - 8)
        lateral_text_y = tag_top + 18
        heading_text_y = lateral_text_y + 16

        label = f"ID:{tag_id}"

        if is_unexpected:
            label += " [UNEXPECTED]"

        cv2.putText(
            frame,
            label,
            (text_x, label_text_y),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.45,
            tag_color,
            1,
            cv2.LINE_AA,
        )

        if detection.lateral is None:
            lateral_text = "Lateral: N/A"

        else:
            lateral_text = f"Lateral: {detection.lateral * 1000.0:+.2f} mm"

        if detection.heading is None:
            heading_text = "Heading: N/A"

        else:
            heading_text = f"Heading: {detection.heading:+.2f} deg"

        cv2.putText(
            frame,
            lateral_text,
            (text_x, lateral_text_y),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.45,
            COLOR_TEXT,
            1,
            cv2.LINE_AA,
        )

        cv2.putText(
            frame,
            heading_text,
            (text_x, heading_text_y),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.45,
            COLOR_TEXT,
            1,
            cv2.LINE_AA,
        )

    if status_lines:
        for index, line in enumerate(status_lines):
            cv2.putText(
                frame,
                line,
                (10, 25 + index * 22),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.55,
                COLOR_HEADING_ARROW,
                1,
                cv2.LINE_AA,
            )

    return frame


# ============================================================================
# MAIN
# ============================================================================
def main():
    ser = None
    camera = None

    try:
        map_data = load_map(MAP_FILE)
        landmark_lookup, tag_lookup = build_map_lookup(map_data)

        if EXPECTED_LANDMARK_ID not in landmark_lookup:
            raise ValueError(
                f"Landmark {EXPECTED_LANDMARK_ID} is not present in the map."
            )

        print(f"Loaded {len(landmark_lookup)} landmarks and {len(tag_lookup)} tags.")

        ser = open_serial()
        print("Serial connected.")
        clear_serial_buffer(ser)

        camera = open_camera()Computation of lateral and 
        print("Camera opened.")

        detector = create_apriltag_detector()
        print("Apriltag detector created.")

        while True:
            rgb_frame = camera.capture_array()
            frame = cv2.cvtColor(rgb_frame, cv2.COLOR_RGB2BGR)
            detections = detect_apriltags(frame, detector)
            enrich_detections(detections)
            draw_frame(frame, detections)
            show_frame(frame)
            cv2.waitKey(1)

    finally:
        if ser is not None:
            try:
                print("Stopping and disabling before exit.")
                stop(ser)
                motor_off(ser)
                ser.close()
            except Exception as exc:
                print(f"Cleanup warning: {exc}")


if __name__ == "__main__":
    main()
