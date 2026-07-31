import cv2
import numpy as np
import math
import json
import time

from pupil_apriltags import Detector
from picamera2 import Picamera2


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


LATERAL_SIGN = 1.0
HEADING_SIGN = -1.0
HEADING_OFFSET_DEG = 0.0

# Display Colors:
COLOR_IMAGE_CENTER = (180, 180, 180)
COLOR_EXPECTED_TAG = (0, 220, 0)
COLOR_SELECTED_TAG = (0, 255, 255)
COLOR_OTHER_TAG = (255, 0, 255)
COLOR_CENTER_LINE = (255, 180, 0)
COLOR_TEXT = (255, 255, 255)


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
        }
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


def center_pose(tag, tag_lookup):
    tag_id = int(tag.tag_id)
    tag_info = tag_lookup.get(tag_id)

    position = tag_info["position"]
    helper_x_m, helper_y_m = HELPER_OFFSETS_M[position]

    # Offset from central tag to detected helper tag,
    # expressed in the tag/landmark frame.
    offset = np.array([helper_x_m, helper_y_m, 0.0], dtype=np.float64)
    t, R = tag_pose(tag)

    # Convert the helper offset from the tag frame into the camera frame.
    offset_camera = R @ offset

    # detected helper = center + rotate offset
    #
    # Therefore:
    # center = detected helper - rotate offset
    center = t - offset_camera

    return center


# Heading Computation:
def tag_yaw_deg(tag):
    _, R = tag_pose(tag)
    yaw_rad = math.atan2(R[1, 0], R[0, 0])
    yaw_deg = math.degrees(yaw_rad)

    return normalize_angle_deg(yaw_deg)


def robot_heading_deg(tag, tag_lookup, landmarks):
    tag_id = int(tag.tag_id)
    landmark_id = tag_lookup[tag_id]["landmark_id"]

    map_heading_deg = float(landmarks[landmark_id].get("heading_offset_deg", 0.0))

    tag_yaw = tag_yaw_deg(tag)
    robot_heading = normalize_angle_deg(
        map_heading_deg + HEADING_SIGN * tag_yaw + HEADING_OFFSET_DEG
    )

    return robot_heading


# Lateral and Heading Errors:
def calculate_errors(tag, tag_lookup, landmarks):
    center = center_pose(tag, tag_lookup)
    lateral_error_m = LATERAL_SIGN * float(center[0])
    robot_heading = robot_heading_deg(tag, tag_lookup, landmarks)
    heading_error_deg = normalize_angle_deg(PATH_HEADING_DEG - robot_heading)

    return lateral_error_m, robot_heading, heading_error_deg


# Visualization:
def draw_frame(frame, detections, expected_tags, selected_id, status_lines):
    height, width = frame.shape[:2]
    image_center_x = int(round(CX))
    image_center_y = int(round(CY))

    image_center = (image_center_x, image_center_y)

    # Full vertical image-center line
    cv2.line(
        frame,
        (0, image_center_y),
        (width - 1, image_center_y),
        COLOR_IMAGE_CENTER,
        thickness=1,
        lineType=cv2.LINE_AA,
    )

    # Full horizontal image-center line
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
            yaw_text = f"Yaw: {tag_yaw_deg(tag):+.2f} deg"

        else:
            yaw_text = "Yaw: N/A"

        cv2.putText(
            frame,
            raw_lateral_text,
            (text_x, label_y + 25),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.42,
            COLOR_TEXT,
            1,
            cv2.LINE_AA,
        )

        cv2.putText(
            frame,
            yaw_text,
            (text_x, label_y + 34),
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


def main():
    camera = None

    try:
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

            status_lines = [f"Expected center: {EXPECTED_ID}"]

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

            now = time.monotonic()

            if now - last_print_time >= PRINT_PERIOD_S:
                if best is None:
                    print(f"Expected {EXPECTED_ID}: no usable tag")

                else:
                    print(
                        f"Expected={EXPECTED_ID} "
                        f"Selected={selected_id} "
                        f"Position={position} "
                        f"Lateral={lateral_error_m * 1000.0:.2f} mm "
                        f"Robot heading={robot_heading:.2f} deg "
                        f"Heading error={heading_error:.2f} deg"
                    )

                last_print_time = now

            key = cv2.waitKey(1) & 0xFF
            if key == ord("q") or key == ord("Q"):
                break

    finally:
        if camera is not None:
            try:
                camera.stop()

            except Exception as exc:
                print(f"Failed to stop camera: {exc}")


if __name__ == "__main__":
    main()
