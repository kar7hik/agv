import cv2
import serial
import time
import math
import json
import networkx as nx
from picamera2 import Picamera2
from pupil_apriltags import Detector

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
SERIAL_BAUD = 115200
ARRIVAL_TIMEOUT_S = 30.0
FIRST_WAYPOINT = 1


# ============================================================================
# CAMERA
# ============================================================================
class Camera:
    def __init__(self, width=FRAME_WIDTH, height=FRAME_HEIGHT):
        self.width = width
        self.height = height
        self.camera = Picamera2()
        self.camera.set_controls(
            {
                "AwbMode": False,
                "ExposureTime": 5000,
                "AnalogueGain": 1.0,
                "AwbEnable": False,
                "ColourGains": (1.7, 1.7),
            }
        )

    def start(self):
        config = self.camera.create_preview_configuration(
            main={"size": (self.width, self.height), "format": "RGB888"}
        )
        self.camera.configure(config)
        self.camera.start()

    def get_frame(self):
        return self.camera.capture_array()

    def release(self):
        self.camera.stop()


# ============================================================================
# APRILTAG DETECTOR
# ============================================================================
class AprilTagDetector:
    def __init__(self):
        self.detector = Detector(
            families=APRILTAG_FAMILY,
            nthreads=4,
            quad_decimate=1.0,
            quad_sigma=0.0,
            refine_edges=True,
        )

    def detect(self, frame):
        gray = cv2.cvtColor(frame, cv2.COLOR_RGB2GRAY)
        return self.detector.detect(
            gray,
            estimate_tag_pose=True,
            camera_params=CAMERA_PARAMS,
            tag_size=TAG_SIZE_M,
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


def compute_heading(detection):
    if detection.pose_R is None:
        return None
    R = detection.pose_R
    return normalize_angle(math.degrees(math.atan2(R[1, 0], R[0, 0])))


def compute_lateral(detection):
    if detection.pose_t is None:
        return None
    return float(detection.pose_t[0][0])


def compute_forward(detection):
    if detection.pose_t is None:
        return None
    return float(detection.pose_t[1][0])


def enrich_detections(detections):
    for d in detections:
        d.heading = compute_heading(d)
        d.lateral = compute_lateral(d)
        d.forward = compute_forward(d)


# ============================================================================
# VIEWER  (with axis crosshair)
# ============================================================================
class Viewer:
    def __init__(self):
        self.window_name = "AGV Navigation"

    def draw(self, frame, detections, status_lines=None):
        h, w = frame.shape[:2]
        cx, cy = w // 2, h // 2

        # --- Axis crosshair ---
        cv2.line(frame, (0, cy), (w, cy), (128, 128, 128), 1)  # horizontal
        cv2.line(frame, (cx, 0), (cx, h), (128, 128, 128), 1)  # vertical
        cv2.circle(frame, (cx, cy), 4, (128, 128, 128), -1)

        # --- Detections ---
        for d in detections:
            corners = d.corners.astype(int)
            for i in range(4):
                cv2.line(
                    frame,
                    tuple(corners[i]),
                    tuple(corners[(i + 1) % 4]),
                    (0, 255, 0),
                    2,
                )

            center = tuple(d.center.astype(int))
            cv2.circle(frame, center, 5, (0, 0, 255), -1)

            x, y = int(corners[0][0]), int(corners[0][1])
            cv2.putText(
                frame,
                f"ID:{d.tag_id}",
                (x, y - 10),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.45,
                (0, 255, 0),
                1,
            )

            info_y = y + 15
            for label, val, fmt in [
                ("Head", d.heading, ".1f"),
                ("Lat", d.lateral, ".3f"),
                ("Fwd", d.forward, ".3f"),
            ]:
                if val is not None:
                    text = f"{label}: {val:{fmt}}"
                else:
                    text = f"{label}: N/A"

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

            # Draw a line from frame center to tag center (lateral visualisation)
            cv2.line(frame, (cx, cy), center, (0, 255, 255), 1)

        # --- Status overlay ---
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

    def show(self, frame):
        cv2.imshow(self.window_name, cv2.cvtColor(frame, cv2.COLOR_RGB2BGR))

    def close(self):
        cv2.destroyAllWindows()


# ============================================================================
# LANDMARK MAP
# ============================================================================
class LandmarkMap:
    def __init__(self, filename):
        with open(filename, "r") as f:
            self.data = json.load(f)
        self.graph = nx.Graph()
        self._build_graph()

    def _build_graph(self):
        for lm in self.data["landmarks"]:
            self.graph.add_node(
                lm["id"],
                id=lm["id"],
                row=lm["row"],
                column=lm["column"],
                name=lm["name"],
                type=lm["type"],
                tags=lm["tags"],
            )
        nodes = list(self.graph.nodes)
        for src in nodes:
            r1, c1 = self.graph.nodes[src]["row"], self.graph.nodes[src]["column"]
            for dst in nodes:
                if src == dst:
                    continue
                r2, c2 = self.graph.nodes[dst]["row"], self.graph.nodes[dst]["column"]
                if abs(r2 - r1) + abs(c2 - c1) == 1:
                    self.graph.add_edge(src, dst, weight=1)

    def get_landmark(self, lid):
        return self.graph.nodes[lid] if self.graph.has_node(lid) else None

    def find_landmark(self, tag_id):
        for lid in self.graph.nodes:
            for pos, tid in self.graph.nodes[lid]["tags"].items():
                if tid == tag_id:
                    return {
                        "landmark": self.graph.nodes[lid],
                        "id": lid,
                        "position": pos,
                    }
        return None

    def find_path(self, start_id, goal_id):
        if not self.graph.has_node(start_id) or not self.graph.has_node(goal_id):
            return []
        try:
            return nx.astar_path(
                self.graph,
                start_id,
                goal_id,
                heuristic=self._heuristic,
                weight="weight",
            )
        except nx.NetworkXNoPath:
            return []

    def _heuristic(self, a, b):
        la, lb = self.get_landmark(a), self.get_landmark(b)
        if la is None or lb is None:
            return float("inf")
        return abs(lb["row"] - la["row"]) + abs(lb["column"] - la["column"])

    def get_heading(self, current_id, target_id):
        cur, tgt = self.get_landmark(current_id), self.get_landmark(target_id)
        if cur is None or tgt is None:
            return None
        dr = tgt["row"] - cur["row"]
        dc = tgt["column"] - cur["column"]
        if dr == -1 and dc == 0:
            return 0.0
        if dr == 0 and dc == 1:
            return 90.0
        if dr == 1 and dc == 0:
            return 180.0
        if dr == 0 and dc == -1:
            return 270.0
        raise ValueError(f"{current_id} and {target_id} are not neighbors.")


# ============================================================================
# SERIAL MANAGER
# ============================================================================
class SerialManager:
    def __init__(self, port, baudrate=SERIAL_BAUD):
        self.serial = serial.Serial(port, baudrate, timeout=0.1)
        time.sleep(2)
        self.serial.reset_input_buffer()
        self.serial.reset_output_buffer()

    def close(self):
        self.serial.close()

    def send_goals(self, desired_heading_deg, lateral_error_mm):
        self.serial.write(f"HEAD {desired_heading_deg:.2f}\n".encode())
        self.serial.write(f"LAT {lateral_error_mm:.1f}\n".encode())

    def correct_heading(self, true_heading_deg):
        """Tell ESP32 its true heading (from tag ground truth)."""
        self.serial.write(f"HCORR {true_heading_deg:.2f}\n".encode())

    def start_move(self):
        self.serial.write(b"MOVE\n")

    def stop_move(self):
        self.serial.write(b"STOP\n")

    def set_speed(self, speed_mm_s):
        self.serial.write(f"SPD {speed_mm_s:.1f}\n".encode())

    def drain(self):
        self.serial.reset_input_buffer()


# ============================================================================
# DETECTION HELPERS
# ============================================================================
def select_best_detection(detections, world, prefer_landmark_id=None):
    """
    From a list of raw detections, pick the best one:
      1. Prefer central tags of the preferred landmark
      2. Then any central tag
      3. Then any tag from the preferred landmark
      4. Then any tag at all
    Returns (detection, landmark_info) or (None, None).
    """
    enriched = []
    for d in detections:
        info = world.find_landmark(d.tag_id)
        if info is not None:
            enriched.append((d, info))

    if not enriched:
        return None, None

    def score(d, info):
        is_center = info["position"] == "center"
        is_preferred = (
            prefer_landmark_id is not None and info["id"] == prefer_landmark_id
        )
        # Lower is better
        s = 0
        if is_center and is_preferred:
            s = 0
        elif is_center:
            s = 1
        elif is_preferred:
            s = 2
        else:
            s = 3
        return s

    enriched.sort(key=lambda x: score(x[0], x[1]))
    return enriched[0]


# ============================================================================
# NAVIGATE ONE SEGMENT
# ============================================================================
def navigate_segment(ser, cam, det, viewer, world, from_node, to_node, velocity_mps):
    """
    Drive from `from_node` to `to_node`.
    - Uses ANY detected tag for lateral correction while driving.
    - Corrects ESP32 IMU whenever a CENTER tag is detected.
    - Only declares arrival when the CENTER tag of `to_node` is detected.
    Returns True on arrival, False on quit/timeout.
    """
    desired_heading = world.get_heading(from_node, to_node)
    seg_label = f"{from_node} -> {to_node}  Head:{desired_heading:.0f}deg"
    print(f"\n[NAV] Segment: {seg_label}")

    ser.start_move()
    ser.set_speed(velocity_mps * 1000.0)

    start_time = time.time()

    while True:
        frame = cam.get_frame()
        raw_detections = det.detect(frame)
        enrich_detections(raw_detections)

        # Pick best detection (prefer center tags of target)
        best_det, best_info = select_best_detection(
            raw_detections, world, prefer_landmark_id=to_node
        )

        arrived = False
        status = [seg_label]

        if best_det is not None and best_info is not None:
            lm_id = best_info["id"]
            pos = best_info["position"]
            status.append(f"Tag:{best_det.tag_id}  LM:{lm_id}  Pos:{pos}")

            # --- IMU correction from any center tag ---
            if pos == "center" and best_det.heading is not None:
                ser.correct_heading(best_det.heading)
                status.append(f"IMU corrected: {best_det.heading:.1f}deg")

            # --- Arrival: only on center tag of target ---
            if lm_id == to_node and pos == "center":
                arrived = True

            # --- Lateral error from best available detection ---
            lateral_mm = (
                best_det.lateral * 1000.0 if best_det.lateral is not None else 0.0
            )
            ser.send_goals(desired_heading, lateral_mm)
            status.append(f"Lat:{lateral_mm:.1f}mm")
        else:
            ser.send_goals(desired_heading, 0.0)
            status.append("No tag visible")

        # Draw + show
        viewer.draw(frame, raw_detections, status)
        viewer.show(frame)

        if arrived:
            print(f"[NAV] Arrived at landmark {to_node} (center tag detected)")
            ser.stop_move()
            time.sleep(0.5)
            return True

        # Quit check
        key = cv2.waitKey(1) & 0xFF
        if key == ord("q"):
            ser.stop_move()
            return False

        # Timeout
        if time.time() - start_time > ARRIVAL_TIMEOUT_S:
            print(f"[NAV] Timeout on segment {seg_label}")
            ser.stop_move()
            return False

        time.sleep(0.02)  # ~50 Hz


# ============================================================================
# MAIN
# ============================================================================
def main():
    cam = Camera()
    det = AprilTagDetector()
    viewer = Viewer()
    world = LandmarkMap(MAP_FILE)

    print("==========================================")
    print("  AGV Navigation System")
    print("==========================================")

    ser = SerialManager(SERIAL_PORT)
    cam.start()
    ser.drain()

    # ------------------------------------------------------------------
    # PHASE 1: Wait for manual alignment at Dock, then drive to first WP
    # ------------------------------------------------------------------
    print("\nPlace the robot at the Dock (Landmark 0).")
    print("Manually align it facing the corridor.")
    print("Press 's' in the camera window to calibrate and start.")
    print("Press 'q' to quit.\n")

    started = False
    while not started:
        frame = cam.get_frame()
        raw = det.detect(frame)
        enrich_detections(raw)
        viewer.draw(frame, raw, ["WAITING: Align robot at Dock, press 's'"])
        viewer.show(frame)

        key = cv2.waitKey(1) & 0xFF
        if key == ord("q"):
            cam.release()
            viewer.close()
            ser.close()
            return
        if key == ord("s"):
            started = True

    print("[INIT] Starting navigation to first waypoint...")

    # Drive Dock -> Landmark 1
    success = navigate_segment(
        ser,
        cam,
        det,
        viewer,
        world,
        from_node=0,
        to_node=FIRST_WAYPOINT,
        velocity_mps=0.05,
    )
    if not success:
        print("Quit during first segment.")
        ser.close()
        cam.release()
        viewer.close()
        return

    # ------------------------------------------------------------------
    # PHASE 2: At first waypoint, wait for target selection
    # ------------------------------------------------------------------
    print("\n" + "=" * 44)
    print("  Robot is at Landmark", FIRST_WAYPOINT)
    print("  Enter the target landmark ID in terminal.")
    print("=" * 44)

    # Keep camera alive while waiting for input
    target_node = None
    while target_node is None:
        frame = cam.get_frame()
        raw = det.detect(frame)
        enrich_detections(raw)
        viewer.draw(frame, raw, ["WAITING: Enter target ID in terminal"])
        viewer.show(frame)
        cv2.waitKey(1)

        try:
            user_input = input("Target landmark ID: ").strip()
            target_node = int(user_input)
            if not world.get_landmark(target_node):
                print(f"  Landmark {target_node} does not exist. Try again.")
                target_node = None
        except ValueError:
            print("  Invalid input. Enter a number.")

    # ------------------------------------------------------------------
    # PHASE 3: Compute path and navigate
    # ------------------------------------------------------------------
    path = world.find_path(FIRST_WAYPOINT, target_node)
    if not path:
        print(f"No path found from {FIRST_WAYPOINT} to {target_node}.")
        ser.close()
        cam.release()
        viewer.close()
        return

    print(f"\nPath: {' -> '.join(map(str, path))}")
    print(f"Segments: {len(path) - 1}\n")

    for i in range(len(path) - 1):
        success = navigate_segment(
            ser,
            cam,
            det,
            viewer,
            world,
            from_node=path[i],
            to_node=path[i + 1],
            velocity_mps=0.05,
        )
        if not success:
            print("Navigation interrupted.")
            break

        # Small pause between segments
        time.sleep(0.5)
    else:
        print("\n*** Navigation complete! ***")

    ser.close()
    cam.release()
    viewer.close()


if __name__ == "__main__":
    main()
