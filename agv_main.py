import cv2
import numpy as np
import math
import json
import time
import serial

from pupil_apriltags import Detector
from picamera2 import Picamera2
from libcamera import Transform


# Camera Parameters:
FRAME_WIDTH = 640
FRAME_HEIGHT = 480

FX = 615.0
FY = 615.0

CX = FRAME_WIDTH / 2.0
CY = FRAME_HEIGHT / 2.0

TAG_SIZE_M = 0.01
CAMERA_PARAMS = (FX, FY, CX, CY)

TAG_FAMILY = "tag36h11"

WINDOW_NAME = "AGV Navigation System"
PRINT_PERIOD_S = 0.5

# Map and Path Parameters:
MAP_FILE = "./maps/workspace.json"
EXPECTED_ID = 6

# Required heading of the current path segment.
PATH_HEADING_DEG = 0.0

# Offsets are from the central tag to each helper tag.
# Expressed in the landmark coordinate frame.
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

GRID_TO_TAG_YAW_DEG = 180.0
HEADING_TRIM_DEG = 0.0

# Display Colors:
COLOR_IMAGE_CENTER = (180, 180, 180)
COLOR_EXPECTED_TAG = (0, 220, 0)
COLOR_SELECTED_TAG = (0, 255, 255)
COLOR_OTHER_TAG = (0, 220, 220)
COLOR_CENTER_LINE = (255, 180, 0)
COLOR_TEXT = (255, 255, 255)


# Serial Parameters:
SERIAL_PORT = "/dev/ttyUSB0"
SERIAL_BAUD = 115200

READ_TIMEOUT_S = 0.1
ACK_TIMEOUT_S = 0.5
CAL_TIMEOUT_S = 20.0

# Drive Parameters:
DRIVE_SPEED_MPS = 0.03


def rotation_z_deg(angle_deg):
    angle_rad = math.radians(angle_deg)
    c = math.cos(angle_rad)
    s = math.sin(angle_rad)

    return np.array(
        [[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]],
        dtype=np.float64,
    )


GRID_TO_TAG_R = rotation_z_deg(GRID_TO_TAG_YAW_DEG)


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


def sync(ser, heading_deg):
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


# Map Loading and Lookups:
def load_map(filename):
    with open(filename, "r", encoding="utf-8") as file:
        map_data = json.load(file)

    if "grid" not in map_data:
        raise ValueError("Map does not contain 'grid'.")

    if "landmarks" not in map_data:
        raise ValueError("Map does not contain 'landmarks'.")

    return map_data


def build_lookups(map_data):
    landmarks = {}
    tags = {}

    for landmark in map_data["landmarks"]:
        landmark_id = int(landmark["id"])

        if landmark_id in landmarks:
            raise ValueError(f"Duplicate landmark ID: {landmark_id}")

        landmarks[landmark_id] = landmark
        landmark_tags = landmark.get("tags")

        if not isinstance(landmark_tags, dict):
            raise ValueError(f"Landmark {landmark_id} does not contain tags.")

        for position, tag_id_value in landmark_tags.items():
            if position not in HELPER_OFFSETS_M:
                raise ValueError(f"Unknown position: {position}")

            tag_id = int(tag_id_value)

            if tag_id in tags:
                previous = tags[tag_id]["landmark_id"]

                raise ValueError(
                    f"Tag ID: {tag_id} is used by both "
                    f"landmark {previous} and landmark {landmark_id}."
                )

            tags[tag_id] = {
                "landmark_id": landmark_id,
                "position": position,
            }

    return landmarks, tags


def get_expected_tags(landmarks, expected_id):
    landmark = landmarks.get(expected_id)

    if landmark is None:
        raise ValueError(f"Expected landmark {expected_id} is not present in the map.")

    landmark_tags = landmark["tags"]
    center_id = int(landmark_tags["center"])

    if center_id != expected_id:
        raise ValueError(
            f"Landmark {expected_id} uses center tag {center_id}. "
            f"Landmark ID and Center tag ID should match."
        )

    expected_tags = {int(tag_id) for tag_id in landmark_tags.values()}

    return expected_tags


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
        },
        transform=Transform(
            hflip=1,
            vflip=1,
        ),
    )

    camera.configure(configuration)
    camera.start()

    # Allowing exposure and white balance to settle.
    time.sleep(1.0)
    return camera


def create_apriltag_detector():
    detector = Detector(
        families=TAG_FAMILY,
        nthreads=4,
        quad_decimate=1.0,
        quad_sigma=0.0,
        refine_edges=True,
        decode_sharpening=0.25,
    )
    return detector


def detect_tags(frame, detector):
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

    detections = detector.detect(
        gray,
        estimate_tag_pose=True,
        camera_params=CAMERA_PARAMS,
        tag_size=TAG_SIZE_M,
    )

    return detections


# Detection Validation:
def valid_tag(tag, expected_tags):
    tag_id = int(tag.tag_id)

    if tag_id not in expected_tags:
        return False

    if tag.hamming != 0:
        return False

    if tag.pose_t is None:
        return False

    if tag.pose_R is None:
        return False

    return True


# Best Tag Selection:
def image_dist_sqaure(tag):
    dx = float(tag.center[0]) - CX
    dy = float(tag.center[1]) - CY

    return dx * dx + dy * dy


def detection_key(tag):
    return (image_dist_sqaure(tag), -float(tag.decision_margin))


def select_best(detections, expected_tags):
    usable = []

    for tag in detections:
        if valid_tag(tag, expected_tags):
            usable.append(tag)

    if not usable:
        return None

    best = min(usable, key=detection_key)

    return best


# Helper Function:
def normalize_angle_deg(angle_deg):
    while angle_deg > 180.0:
        angle_deg -= 360.0

    while angle_deg < -180.0:
        angle_deg += 360.0

    return angle_deg


# Tag and Central Pose:
def tag_pose(tag):
    t = np.asarray(tag.pose_t, dtype=np.float64).reshape(3)
    R = np.asarray(tag.pose_R, dtype=np.float64).reshape(3, 3)

    return t, R


# def center_pose(tag, tag_lookup):
#     tag_id = int(tag.tag_id)
#     tag_info = tag_lookup.get(tag_id)

#     position = tag_info["position"]
#     helper_x_m, helper_y_m = HELPER_OFFSETS_M[position]

#     # Offset from central tag to detected helper tag,
#     # expressed in the tag/landmark frame.
#     offset = np.array([helper_x_m, helper_y_m, 0.0], dtype=np.float64)
#     t, R = tag_pose(tag)

#     # Landmark-grid frame -> Camera frame.
#     R_camera_grid = R @ GRID_TO_TAG_R

#     # Center-to-helper displacement expressed in camera coordinates.
#     offset_camera = R_camera_grid @ offset

#     # detected helper = center + rotate offset
#     #
#     # Therefore:
#     # center = detected helper - rotate offset
#     center = t - offset_camera

#     return center


# Heading Computation:
def tag_heading_deg(tag):
    _, R = tag_pose(tag)
    R_camera_grid = R @ GRID_TO_TAG_R

    heading_rad = math.atan2(R_camera_grid[1, 0], R_camera_grid[0, 0])
    heading_deg = math.degrees(heading_rad)

    return normalize_angle_deg(heading_deg)


def robot_heading_deg(tag, tag_lookup, landmarks):
    tag_id = int(tag.tag_id)
    landmark_id = tag_lookup[tag_id]["landmark_id"]

    map_heading_deg = float(landmarks[landmark_id].get("heading_offset_deg", 0.0))
    tag_heading = tag_heading_deg(tag)

    # A stationary floor tag appears to rotate opposite to the robot.
    robot_heading = normalize_angle_deg(
        map_heading_deg + tag_heading + HEADING_TRIM_DEG
    )

    return robot_heading


# Lateral and Heading Errors:
def lateral_error(tag, tag_lookup):
    tag_id = int(tag.tag_id)
    position = tag_lookup[tag_id]["position"]

    tag_x = float(tag.pose_t[0][0])
    helper_x, _ = HELPER_OFFSETS_M[position]

    return tag_x - helper_x


def calculate_errors(tag, tag_lookup, landmarks):
    lateral_error_m = lateral_error(tag, tag_lookup)
    robot_heading = robot_heading_deg(tag, tag_lookup, landmarks)
    heading_error_deg = normalize_angle_deg(PATH_HEADING_DEG - robot_heading)

    return lateral_error_m, robot_heading, heading_error_deg


# Visualization:
def draw_frame(frame, detections, expected_tags, selected_id, status_lines):
    height, width = frame.shape[:2]
    image_center_x = int(round(CX))
    image_center_y = int(round(CY))

    image_center = (image_center_x, image_center_y)

    # Full horizontal image-center line
    cv2.line(
        frame,
        (0, image_center_y),
        (width - 1, image_center_y),
        COLOR_IMAGE_CENTER,
        thickness=1,
        lineType=cv2.LINE_AA,
    )

    # Full vertical image-center line
    cv2.line(
        frame,
        (image_center_x, 0),
        (image_center_x, height - 1),
        COLOR_IMAGE_CENTER,
        thickness=1,
        lineType=cv2.LINE_AA,
    )

    # Image-center marker
    cv2.circle(
        frame,
        image_center,
        4,
        COLOR_IMAGE_CENTER,
        thickness=-1,
        lineType=cv2.LINE_AA,
    )

    for tag in detections:
        tag_id = int(tag.tag_id)

        corners = np.rint(tag.corners).astype(np.int32)
        center = (int(round(tag.center[0])), int(round(tag.center[1])))

        is_selected = selected_id is not None and selected_id == tag_id

        if is_selected:
            color = COLOR_SELECTED_TAG
            thickness = 3

        elif tag_id in expected_tags:
            color = COLOR_EXPECTED_TAG
            thickness = 2

        else:
            color = COLOR_OTHER_TAG
            thickness = 2

        cv2.polylines(
            frame,
            [corners],
            isClosed=True,
            color=color,
            thickness=thickness,
            lineType=cv2.LINE_AA,
        )

        cv2.circle(
            frame,
            center,
            4,
            color,
            thickness=-1,
            lineType=cv2.LINE_AA,
        )

        # Draw the image-center line only for the selected tag
        if is_selected:
            cv2.line(
                frame,
                image_center,
                center,
                COLOR_CENTER_LINE,
                thickness=2,
                lineType=cv2.LINE_AA,
            )

        tag_left = int(np.min(corners[:, 0]))
        tag_top = int(np.min(corners[:, 1]))

        text_x = max(5, tag_left + 15)
        label_y = max(18, tag_top - 8)

        label = f"ID:{tag_id}"

        if is_selected:
            label += " [SELECTED]"

        cv2.putText(
            frame,
            label,
            (text_x, label_y),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.45,
            color,
            1,
            cv2.LINE_AA,
        )

        if tag.pose_t is not None:
            raw_lateral_mm = float(tag.pose_t[0][0]) * 1000.0
            raw_lateral_text = f"Lat: {raw_lateral_mm:.2f} mm"

        else:
            raw_lateral_text = "Lat: N/A"

        if tag.pose_R is not None:
            yaw_text = f"Yaw: {tag_heading_deg(tag):+.2f} deg"

        else:
            yaw_text = "Yaw: N/A"

        cv2.putText(
            frame,
            raw_lateral_text,
            (text_x, label_y + 30),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.42,
            COLOR_TEXT,
            1,
            cv2.LINE_AA,
        )

        cv2.putText(
            frame,
            yaw_text,
            (text_x, label_y + 50),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.42,
            COLOR_TEXT,
            1,
            cv2.LINE_AA,
        )

    for index, line in enumerate(status_lines):
        cv2.putText(
            frame,
            line,
            (10, 25 + index * 22),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.35,
            COLOR_SELECTED_TAG,
            1,
            cv2.LINE_AA,
        )

    return frame


def send_keyboard_commands(ser, imu_ready, motors_enabled, best, lateral_error_m):
    key = cv2.waitKey(1)

    if key == ord("q") or key == ord("Q"):
        return False


def main():
    camera = None
    ser = None

    imu_ready = False
    motors_enabled = False
    serial_enabled = True

    try:
        if serial_enabled:
            ser = open_serial()
            print("Serial port ready.")
            clear_serial_buffer(ser)

            if not ping(ser):
                raise RuntimeError("ESP32 did not acknowledge PING.")

            print("ESP32 communication ready.")

        map_data = load_map(MAP_FILE)
        landmarks, tag_lookup = build_lookups(map_data)

        expected_tags = get_expected_tags(landmarks, EXPECTED_ID)

        print(f"Expected center tag: {EXPECTED_ID}")
        print(f"Accepted tag IDs: {sorted(expected_tags)}")

        camera = open_camera()
        detector = create_apriltag_detector()
        print("Camera and Apriltag detector ready.")

        last_print_time = 0.0

        while True:
            rgb_frame = camera.capture_array()
            frame = cv2.cvtColor(rgb_frame, cv2.COLOR_RGB2BGR)
            detections = detect_tags(frame, detector)
            best = select_best(detections, expected_tags)

            selected_id = None
            lateral_error_m = None
            robot_heading = None
            heading_error = None
            driving = False

            status_lines = [f"Expected center: {EXPECTED_ID}"]

            if motors_enabled:
                status_lines.append("Motors: ENABLED")

            else:
                status_lines.append("Motors: DISABLED")

            if imu_ready:
                status_lines.append("IMU: READY")

            else:
                status_lines.append("IMU: NOT READY")

            if best is not None:
                selected_id = int(best.tag_id)
                lateral_error_m, robot_heading, heading_error = calculate_errors(
                    best, tag_lookup, landmarks
                )

                position = tag_lookup[selected_id]["position"]
                status_lines.append(
                    f"Central Lateral error: {lateral_error_m * 1000.0:.2f} mm"
                )
                status_lines.append(f"Robot heading: {robot_heading:.2f} deg")
                status_lines.append(f"Heading error: {heading_error:.2f} deg")

            else:
                status_lines.append("Selected: NONE")

            draw_frame(frame, detections, expected_tags, selected_id, status_lines)
            cv2.imshow(WINDOW_NAME, frame)

            # now = time.monotonic()

            # if now - last_print_time >= PRINT_PERIOD_S:
            #     if best is None:
            #         print(f"Expected {EXPECTED_ID}: no usable tag")

            #     else:
            #         print(
            #             f"Expected={EXPECTED_ID} "
            #             f"Selected={selected_id} "
            #             f"Position={position} "
            #             f"Lateral={lateral_error_m * 1000.0:.2f} mm "
            #             f"Robot heading={robot_heading:.2f} deg "
            #             f"Heading error={heading_error:.2f} deg"
            #         )

            #     last_print_time = now

            if serial_enabled:
                key = cv2.waitKey(1)

                if key == ord("q") or key == ord("Q"):
                    return False

                elif key == ord("p") or key == ord("P"):
                    ping(ser)

                elif key == ord("c") or key == ord("C"):
                    print("Preparing for IMU calibration...")
                    stop_ok = stop(ser)
                    motor_off_ok = motor_off(ser)

                    if motor_off_ok:
                        motors_enabled = False

                    if not stop_ok or not motor_off_ok:
                        print("Failed to stop motors before IMU calibration.")

                    else:
                        print("Keep the robot completely stationary...")
                        calibration_ok = cal_imu(ser)

                        if not calibration_ok:
                            imu_ready = False
                            print("IMU calibration failed.")

                        else:
                            zero_ok = zero_imu(ser)
                            imu_ready = zero_ok

                            if zero_ok:
                                print("IMU calibration complete.")
                            else:
                                print("IMU zeroing failed.")

                elif key == ord("m") or key == ord("M"):
                    if not imu_ready:
                        print("Motors not enabled. Enable IMU first.")

                    else:
                        motors_enabled = motor_on(ser)

                        if motors_enabled:
                            print("Motors enabled.")
                        else:
                            print("Failed to enable motors.")

                elif key == ord("t") or key == ord("T"):
                    status(ser)

                elif key == ord("z") or key == ord("Z"):
                    if imu_ready:
                        zero_imu(ser)
                        print("IMU zeroed.")

                    else:
                        print("IMU not ready.")

                elif key == ord("f") or key == ord("F"):
                    stop(ser)

                    if motor_off(ser):
                        driving = False
                        motors_enabled = False
                        print("Motors disabled.")

                elif key == ord("s") or key == ord("S"):
                    if best is None:
                        print("SYNC rejected: no usable tag")

                    elif robot_heading is None:
                        print("SYNC rejected: no heading")

                    else:
                        print(f"SYNC to {robot_heading:.2f} deg")
                        sync_ok = sync(ser, robot_heading)

                        if sync_ok:
                            print("SYNC OK")
                        else:
                            print("SYNC rejected")

                elif key == ord("d") or key == ord("D"):
                    if driving:
                        print("DRIVE rejected: already driving")

                    if not imu_ready:
                        print("DRIVE rejected: IMU not ready")

                    elif not motors_enabled:
                        print("DRIVE rejected: motors not enabled")

                    elif best is None:
                        print("DRIVE rejected: no usable tag")

                    elif lateral_error_m is None or robot_heading is None:
                        print("DRIVE rejected: Observation is incomplete")

                    else:
                        sync_ok = sync(ser, robot_heading)

                        if sync_ok:
                            print(
                                f"DRIVE_SPEED_MPS={DRIVE_SPEED_MPS} PATH_HEADING_DEG={PATH_HEADING_DEG} lateral_error_m={lateral_error_m}"
                            )
                            drive_ok = drive(
                                ser, DRIVE_SPEED_MPS, PATH_HEADING_DEG, lateral_error_m
                            )

                            if drive_ok:
                                driving = True
                                print("DRIVE OK")
                            else:
                                print("DRIVE rejected")
                        else:
                            print("SYNC rejected. DRIVE cancelled.")

            else:
                key = cv2.waitKey(1)
                if key == ord("q") or key == ord("Q"):
                    return False

    finally:
        if ser is not None:
            try:
                print("Stopping and disabling robot.")
                stop(ser)
                motor_off(ser)
                ser.close()

            except Exception as exc:
                print(f"Failed to stop robot: {exc}")

        if camera is not None:
            try:
                camera.stop()

            except Exception as exc:
                print(f"Failed to stop camera: {exc}")

        cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
