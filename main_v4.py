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

# Serial Parameters
SERIAL_PORT = "/dev/ttyUSB0"
SERIAL_BAUD = 115200

READ_TIMEOUT_S = 0.1
ACK_TIMEOUT_S = 0.5
CAL_TIMEOUT_S = 20.0


# Drive Parameters
DRIVE_SPEED_MPS = 0.030


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


def create_apriltag_detector():
    return Detector(
        families=APRILTAG_FAMILY,
        nthreads=4,
        quad_decimate=1.0,
        quad_sigma=0.0,
        refine_edges=1,
        decode_sharpening=0.25,
        debug=0,
    )


def detect_apriltags(frame, detector):
    gray_frame = cv2.cvtColor(frame, cv2.COLOR_RGB2GRAY)

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


def draw_apriltag_detections(frame, detections, tag_lookup):
    display_frame = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)

    for detection in detections:
        tag_id = int(detection.tag_id)

        corners = np.rint(detection.corners).astype(np.int32)
        center_x = int(round(detection.center[0]))
        center_y = int(round(detection.center[1]))

        cv2.polylines(
            display_frame,
            [corners],
            isClosed=True,
            color=(0, 255, 0),
            thickness=2,
        )

        cv2.circle(
            display_frame,
            (center_x, center_y),
            radius=5,
            color=(0, 0, 255),
            thickness=-1,
        )

        tag_info = tag_lookup.get(tag_id)

        if tag_info is None:
            label = f"ID: {tag_id} UNKNOWN"
        else:
            landmark_id = tag_info["landmark_id"]
            position = tag_info["position"]
            label = f"ID: {tag_id} ({landmark_id} {position})"

        text_position = (int(corners[0][0]), int(corners[0][1]) - 8)

        cv2.putText(
            display_frame,
            label,
            text_position,
            cv2.FONT_HERSHEY_SIMPLEX,
            0.45,
            (0, 0, 255),
            cv2.LINE_AA,
        )

    return display_frame


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

        camera = open_camera()
        print("Camera opened.")

        detector = create_apriltag_detector()
        print("Apriltag detector created.")

        while True:
            frame = camera.capture_array()
            detections = detect_apriltags(frame, detector)
            display_frame = draw_apriltag_detections(frame, detections, tag_lookup)
            cv2.imshow(WINDOW_NAME, display_frame)

            key = cv2.waitKey(1) & 0xFF

            if key == ord("q") or key == 27:
                break

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
