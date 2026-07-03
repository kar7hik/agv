import cv2
import serial
import time
import math
import json
import networkx as nx
from picamera2 import Picamera2
from pupil_apriltags import Detector
import numpy as np

# ============================================================================
# CONFIGURATION
# ============================================================================
FRAME_WIDTH = 640
FRAME_HEIGHT = 480
FX = 615.0
FY = 615.0
CX = FRAME_WIDTH / 2.0
CY = FRAME_HEIGHT / 2.0
TAG_SIZE_M = 0.01
CAMERA_PARAMS = (FX, FY, CX, CY)
APRILTAG_FAMILY = "tag36h11"

MAP_FILE = "./maps/testbed.json"
SERIAL_PORT = "/dev/ttyUSB0"

HELPER_SPACING_M = 0.015
CORRECTION_DISTANCE_MM = 350.0
SEGMENT_TIMEOUT_S = 60.0


# ============================================================================
# CAMERA & DETECTOR
# ============================================================================
def create_camera():
    cam = Picamera2()
    cam.set_controls(
        {
            "AwbMode": False,
            "ExposureTime": 5000,
            "AnalogueGain": 1.0,
            "AwbEnable": False,
            "ColourGains": (1.7, 1.7),
        }
    )
    config = cam.create_preview_configuration(
        main={"size": (FRAME_WIDTH, FRAME_HEIGHT), "format": "RGB888"}
    )
    cam.configure(config)
    cam.start()
    return cam


def get_frame(cam):
    return cam.capture_array()


def create_detector():
    return Detector(
        families=APRILTAG_FAMILY,
        nthreads=4,
        quad_decimate=1.0,
        quad_sigma=0.0,
        refine_edges=True,
    )


def detect_tags(detector, frame):
    gray = cv2.cvtColor(frame, cv2.COLOR_RGB2GRAY)
    return detector.detect(
        gray, estimate_tag_pose=True, camera_params=CAMERA_PARAMS, tag_size=TAG_SIZE_M
    )


# ============================================================================
# GEOMETRY
# ============================================================================
def normalize_angle(angle):
    while angle > 180:
        angle -= 360
    while angle < -180:
        angle += 360
    return angle


def compute_tag_heading(detection):
    if detection.pose_R is None:
        return None
    return normalize_angle(
        math.degrees(math.atan2(detection.pose_R[1, 0], detection.pose_R[0, 0]))
    )


def compute_tag_lateral(detection):
    return float(detection.pose_t[0][0]) if detection.pose_t is not None else None


def enrich_detections(detections):
    for d in detections:
        d.heading = compute_tag_heading(d)
        d.lateral = compute_tag_lateral(d)


def get_tag_x_offset_from_center(position_name):
    offsets = {
        "center": 0.0,
        "north": 0.0,
        "south": 0.0,
        "east": HELPER_SPACING_M,
        "west": -HELPER_SPACING_M,
        "north_east": HELPER_SPACING_M,
        "north_west": -HELPER_SPACING_M,
        "south_east": HELPER_SPACING_M,
        "south_west": -HELPER_SPACING_M,
    }
    return offsets.get(position_name, 0.0)


def correct_lateral_to_center(detection, position_name):
    if (
        detection.pose_t is None
        or detection.pose_R is None
        or detection.lateral is None
    ):
        return detection.lateral
    if position_name == "center":
        return detection.lateral
    x_offset_lm = get_tag_x_offset_from_center(position_name)
    offset_cam = detection.pose_R.T @ np.array([x_offset_lm, 0.0, 0.0])
    return detection.lateral - offset_cam[0]


# ============================================================================
# MAP
# ============================================================================
def load_landmark_map(filename):
    with open(filename, "r") as f:
        data = json.load(f)
    graph = nx.Graph()
    for lm in data["landmarks"]:
        graph.add_node(
            lm["id"],
            id=lm["id"],
            row=lm["row"],
            column=lm["column"],
            name=lm["name"],
            type=lm["type"],
            tags=lm["tags"],
        )
    nodes = list(graph.nodes)
    for src in nodes:
        r1, c1 = graph.nodes[src]["row"], graph.nodes[src]["column"]
        for dst in nodes:
            if src == dst:
                continue
            r2, c2 = graph.nodes[dst]["row"], graph.nodes[dst]["column"]
            if abs(r2 - r1) + abs(c2 - c1) == 1:
                graph.add_edge(src, dst, weight=1)
    return graph, data


def find_landmark_by_tag(graph, tag_id):
    for lid in graph.nodes:
        for pos, tid in graph.nodes[lid]["tags"].items():
            if tid == tag_id:
                return {"landmark": graph.nodes[lid], "id": lid, "position": pos}
    return None


def find_path(graph, start_id, goal_id):
    if not graph.has_node(start_id) or not graph.has_node(goal_id):
        return []

    def heuristic(a, b):
        return abs(graph.nodes[b]["row"] - graph.nodes[a]["row"]) + abs(
            graph.nodes[b]["column"] - graph.nodes[a]["column"]
        )

    try:
        return nx.astar_path(
            graph, start_id, goal_id, heuristic=heuristic, weight="weight"
        )
    except nx.NetworkXNoPath:
        return []


def get_heading(graph, current_id, target_id):
    cur, tgt = graph.nodes[current_id], graph.nodes[target_id]
    dr, dc = tgt["row"] - cur["row"], tgt["column"] - cur["column"]
    if dr == 1 and dc == 0:
        return 0.0
    if dr == 0 and dc == 1:
        return 90.0
    if dr == -1 and dc == 0:
        return 180.0
    if dr == 0 and dc == -1:
        return 270.0
    raise ValueError(f"{current_id} and {target_id} are not neighbors.")


# ============================================================================
# SERIAL & ROTATION
# ============================================================================
def create_serial(port, baudrate=115200):
    ser = serial.Serial(port, baudrate, timeout=0.1)
    time.sleep(2)
    ser.reset_input_buffer()
    ser.reset_output_buffer()
    return ser


def send_goals(ser, desired_heading_deg, lateral_error_mm):
    ser.write(f"HEAD {desired_heading_deg:.2f}\n".encode())
    ser.write(f"LAT {lateral_error_mm:.1f}\n".encode())


def correct_heading(ser, true_heading_deg):
    ser.write(f"HCORR {true_heading_deg:.2f}\n".encode())


def start_move(ser):
    ser.write(b"MOVE\n")


def stop_move(ser):
    ser.write(b"STOP\n")


def set_speed(ser, speed_mm_s):
    ser.write(f"SPD {speed_mm_s:.1f}\n".encode())


def rotate_to(ser, target_heading_deg):
    """Command ESP32 to rotate in place to target heading."""
    ser.write(f"ROT {target_heading_deg:.2f}\n".encode())


def wait_for_rotation(ser, timeout=10.0):
    """Wait until ESP32 reports rotation is complete."""
    start = time.time()
    while time.time() - start < timeout:
        if ser.in_waiting > 0:
            line = ser.readline().decode("utf-8", errors="ignore").strip()
            if "ROT_DONE" in line:
                return True
        time.sleep(0.05)
    return False


# ============================================================================
# VIEWER
# ============================================================================
def draw_frame(frame, detections, status_lines=None, highlight_ids=None):
    if highlight_ids is None:
        highlight_ids = set()
    h, w = frame.shape[:2]
    cx, cy = w // 2, h // 2
    cv2.line(frame, (0, cy), (w, cy), (128, 128, 128), 1)
    cv2.line(frame, (cx, 0), (cx, h), (128, 128, 128), 1)
    cv2.circle(frame, (cx, cy), 4, (128, 128, 128), -1)

    for d in detections:
        corners = d.corners.astype(int)
        is_unexpected = d.tag_id in highlight_ids
        color = (0, 0, 255) if is_unexpected else (0, 255, 0)
        thickness = 3 if is_unexpected else 2
        for i in range(4):
            cv2.line(
                frame, tuple(corners[i]), tuple(corners[(i + 1) % 4]), color, thickness
            )
        center = tuple(d.center.astype(int))
        cv2.circle(frame, center, 5, color, -1)
        x, y = int(corners[0][0]), int(corners[0][1])
        label = f"ID:{d.tag_id}" + (" [UNEXPECTED]" if is_unexpected else "")
        cv2.putText(
            frame, label, (x, y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.45, color, thickness
        )
        info_y = y + 15
        for lbl, val, fmt in [("Head", d.heading, ".1f"), ("Lat", d.lateral, ".3f")]:
            text = f"{lbl}: {val:{fmt}}" if val is not None else f"{lbl}: N/A"
            cv2.putText(
                frame,
                text,
                (x, info_y),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.4,
                (255, 255, 255),
                1,
            )
            info_y += 14
        cv2.line(frame, (cx, cy), center, color, 1)

    if status_lines:
        for i, line in enumerate(status_lines):
            cv2.putText(
                frame,
                line,
                (10, 25 + i * 22),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.55,
                (0, 255, 255),
                2,
            )
    return frame


def show_frame(frame):
    cv2.imshow("AGV Navigation", cv2.cvtColor(frame, cv2.COLOR_RGB2BGR))


# ============================================================================
# DETECTION SELECTION (Unified for both Pass-through and Stopping)
# ============================================================================
def select_best_detection(
    detections, graph, target_landmark_id, start_landmark_id, has_seen_target
):
    """
    Select the best tag. Returns (best_det, best_info, unexpected_info, new_has_seen_target).
    - Tolerates tags from start_landmark until we see the first tag from target_landmark.
    - Flags any tag from a different cluster as unexpected.
    - Prefers center tags, but accepts side/corner tags if center isn't visible.
    """
    best_det, best_info, unexpected_info = None, None, None
    best_score = 99
    new_has_seen_target = has_seen_target

    for d in detections:
        info = find_landmark_by_tag(graph, d.tag_id)
        if info is None:
            if unexpected_info is None:
                unexpected_info = {"tag_id": d.tag_id, "reason": "unknown_tag"}
            continue

        cluster_id, pos = info["id"], info["position"]

        if cluster_id == target_landmark_id:
            new_has_seen_target = True
        elif cluster_id == start_landmark_id and not has_seen_target:
            continue  # Still leaving start cluster
        else:
            if unexpected_info is None:
                unexpected_info = {
                    "tag_id": d.tag_id,
                    "cluster_id": cluster_id,
                    "reason": "different_cluster",
                }
            continue

        # Tag is from target cluster. Score it.
        if pos == "center":
            score = 0
        elif pos in ("north", "south", "east", "west"):
            score = 1
        else:
            score = 2

        if score < best_score:
            best_score = score
            best_det, best_info = d, info
            if score == 0:
                break  # Can't do better than center

    return best_det, best_info, unexpected_info, new_has_seen_target


# ============================================================================
# SEGMENT NAVIGATION
# ============================================================================
def navigate_segment(
    ser, cam, det, graph, from_node, to_node, velocity_mps, is_final_target=False
):
    desired_heading = get_heading(graph, from_node, to_node)
    direction_name = {0.0: "NORTH", 90.0: "EAST", 180.0: "SOUTH", 270.0: "WEST"}.get(
        desired_heading, "?"
    )
    print(
        f"\n[SEGMENT] {from_node} -> {to_node} | {direction_name} ({desired_heading:.0f}°)"
    )

    start_move(ser)
    start_time = time.time()
    last_log_time = 0.0
    tags_seen = set()
    has_seen_target = False

    while True:
        frame = get_frame(cam)
        raw = detect_tags(det, frame)
        enrich_detections(raw)
        elapsed = time.time() - start_time
        status = [f"{from_node}->{to_node} {desired_heading:.0f}°"]

        best_det, best_info, unexpected, has_seen_target = select_best_detection(
            raw, graph, to_node, from_node, has_seen_target
        )

        # Safety check
        if unexpected is not None:
            stop_move(ser)
            print(f"\n[!!!] SAFETY STOP — Unexpected tag {unexpected['tag_id']}")
            draw_frame(
                frame,
                raw,
                [f"SAFETY: {unexpected['tag_id']}"],
                highlight_ids={unexpected["tag_id"]},
            )
            show_frame(frame)
            while True:
                key = cv2.waitKey(100) & 0xFF
                if key == ord("q"):
                    return False
                if key != 255:
                    print("  Resuming...")
                    start_move(ser)
                    break
            continue

        if best_det is not None and best_info is not None:
            tag_id, pos = best_det.tag_id, best_info["position"]
            tags_seen.add(tag_id)
            status.append(f"Tag:{tag_id} {pos}")

            if pos == "center" and best_det.heading is not None:
                correct_heading(ser, best_det.heading)
                status.append(f"IMU->{best_det.heading:.1f}°")

            corrected_lat = correct_lateral_to_center(best_det, pos)
            lat_mm = corrected_lat * 1000.0 if corrected_lat is not None else 0.0
            send_goals(ser, desired_heading, lat_mm)
            status.append(f"Lat:{lat_mm:.1f}mm")

            if has_seen_target:
                stop_move(ser)
                break  # Reached the target cluster

            if elapsed - last_log_time >= 1.0:
                last_log_time = elapsed
                print(
                    f"  [{elapsed:5.1f}s] Tag:{tag_id:3d} {pos:11s} Lat:{lat_mm:+7.1f}mm"
                )
        else:
            send_goals(ser, desired_heading, 0.0)
            status.append("NO TAG")
            if elapsed - last_log_time >= 2.0:
                last_log_time = elapsed
                print(f"  [{elapsed:5.1f}s] No tag visible")

        draw_frame(frame, raw, status)
        show_frame(frame)

        key = cv2.waitKey(1) & 0xFF
        if key == ord("q"):
            stop_move(ser)
            return False
        if elapsed > SEGMENT_TIMEOUT_S:
            print(f"  [!] TIMEOUT")
            stop_move(ser)
            return False
        time.sleep(0.02)

    # We are now stopped at the target cluster.
    if is_final_target:
        print("  [PHASE 2] Final alignment check (looking for center tag)...")
        # Wait up to 5 seconds to see a center tag for a final IMU correction
        for _ in range(100):
            frame = get_frame(cam)
            raw = detect_tags(det, frame)
            enrich_detections(raw)
            for d in raw:
                info = find_landmark_by_tag(graph, d.tag_id)
                if info and info["id"] == to_node and info["position"] == "center":
                    if d.heading is not None:
                        correct_heading(ser, d.heading)
                        print(f"  [OK] Final IMU corrected to {d.heading:.2f}°")
                    break
            time.sleep(0.05)
        print(f"  [OK] Arrived at landmark {to_node}")
    else:
        print(f"  [OK] Passed through landmark {to_node}")

    return True


# ============================================================================
# MAIN
# ============================================================================
def main():
    print("=" * 60)
    print("  AGV Navigation System")
    print("=" * 60)

    graph, _ = load_landmark_map(MAP_FILE)
    cam = create_camera()
    det = create_detector()
    ser = create_serial(SERIAL_PORT)

    # ---------- Phase 1: Manual alignment ----------
    print("\n[PHASE 1] Manual Alignment")
    print("  1. Place robot at Dock (Landmark 0)")
    print("  2. Ensure CENTER tag (ID 0) is visible")
    print("  3. Physically align robot facing North")
    print("  4. Press 's' to initialize and start")
    started = False
    while not started:
        frame = get_frame(cam)
        raw = detect_tags(det, frame)
        enrich_detections(raw)
        draw_frame(frame, raw, ["PHASE 1: Align at Dock, press 's'"])
        show_frame(frame)
        key = cv2.waitKey(1) & 0xFF
        if key == ord("q"):
            cam.stop()
            cv2.destroyAllWindows()
            ser.close()
            return
        if key == ord("s"):
            dock_center = next(
                (
                    d
                    for d in raw
                    if find_landmark_by_tag(graph, d.tag_id)
                    in [{"id": 0, "position": "center"}]
                ),
                None,
            )
            # Simplified check for dock center
            for d in raw:
                info = find_landmark_by_tag(graph, d.tag_id)
                if info and info["id"] == 0 and info["position"] == "center":
                    dock_center = d
                    break

            if dock_center is None:
                print("  [!] Dock center tag (ID 0) not visible.")
                continue
            if dock_center.heading is None:
                print("  [!] Cannot compute tag heading.")
                continue
            correct_heading(ser, dock_center.heading)
            print(f"  [OK] IMU initialized to {dock_center.heading:.2f}°")
            started = True

    # ---------- Phase 2: Target selection ----------
    print("\n[PHASE 2] Target Selection")
    target_node = None
    while target_node is None:
        frame = get_frame(cam)
        raw = detect_tags(det, frame)
        enrich_detections(raw)
        draw_frame(frame, raw, ["PHASE 2: Enter target ID"])
        show_frame(frame)
        cv2.waitKey(1)
        try:
            target_node = int(input("Target landmark ID (1-20): ").strip())
            if not graph.has_node(target_node) or target_node == 0:
                print("  [!] Invalid ID.")
                target_node = None
        except ValueError:
            print("  [!] Invalid input.")

    # ---------- Phase 3: Navigation ----------
    path = find_path(graph, 0, target_node)
    if not path:
        print(f"\n[ERROR] No path from 0 to {target_node}.")
        cam.stop()
        cv2.destroyAllWindows()
        ser.close()
        return

    print(f"\n[PHASE 3] Navigation")
    print(f"  Path: {' -> '.join(map(str, path))}")

    velocity_mps = 0.05
    set_speed(ser, velocity_mps * 1000.0)

    for i in range(len(path) - 1):
        from_node, to_node = path[i], path[i + 1]
        is_final = to_node == target_node
        desired_heading = get_heading(graph, from_node, to_node)

        # 1. ROTATE to the correct heading for this segment
        print(
            f"\n[ROTATE] Turning to {desired_heading:.0f}° for segment {from_node}->{to_node}"
        )
        rotate_to(ser, desired_heading)
        if not wait_for_rotation(ser):
            print("  [!] Rotation timeout")
        time.sleep(0.3)  # Brief pause after rotation

        # 2. NAVIGATE the segment
        success = navigate_segment(
            ser, cam, det, graph, from_node, to_node, velocity_mps, is_final
        )
        if not success:
            print("\n[ABORT] Navigation interrupted.")
            break

        if not is_final:
            time.sleep(0.2)  # Brief pause before next segment
    else:
        print("\n" + "=" * 60)
        print(f"  *** NAVIGATION COMPLETE: Reached Landmark {target_node} ***")
        print("=" * 60)

    cam.stop()
    cv2.destroyAllWindows()
    ser.close()


if __name__ == "__main__":
    main()
