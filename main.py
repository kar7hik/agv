import cv2

from camera import Camera
from detector import AprilTagDetector
from viewer import Viewer

import geometry

from landmark_map import LandmarkMap
from localization import Localization
from navigation import Navigation
from serial_manager import SerialManager


def main():
    camera = Camera()
    detector = AprilTagDetector()
    viewer = Viewer()

    world = LandmarkMap("./maps/testbed.json")

    localization = Localization(world)
    navigation = Navigation(world)
    navigation.set_target(1)
    navigation.set_velocity(0.05)  # 0.05 m/s = 50 mm/s

    serial = SerialManager("/dev/ttyUSB0")

    camera.start()

    started = False
    last_current_landmark = None

    print("==========================================")
    print("Robot Ready")
    print("Place the robot at Landmark 0.")
    print("Align it with the corridor (facing North).")
    print("Press 's' to start.")
    print("Press 'q' to quit.")
    print("==========================================")

    try:
        while True:
            frame = camera.get_frame()
            detections = detector.detect(frame)
            geometry.update(detections)
            localization.update(detections)
            navigation.update(localization)

            if started:
                # --- Trigger MOVE/STOP on landmark transitions ---
                if (
                    navigation.current is not None
                    and navigation.current != last_current_landmark
                ):
                    if navigation.current == navigation.target:
                        print(f"Reached target landmark {navigation.current}.")
                        serial.stop_move()
                    elif navigation.next is not None:
                        print(
                            f"Arrived at {navigation.current}, moving to {navigation.next}"
                        )
                        serial.start_move()
                    last_current_landmark = navigation.current

                # --- Send goals to ESP32 (it computes corrections internally) ---
                if navigation.desired_heading is not None:
                    # Lateral error: convert meters → mm, default to 0 if tag lost
                    lat_mm = (
                        (navigation.lateral_error * 1000.0)
                        if navigation.lateral_error is not None
                        else 0.0
                    )

                    serial.send_goals(
                        desired_heading_deg=navigation.desired_heading,
                        lateral_error_mm=lat_mm,
                    )

            viewer.draw(frame, detections)
            viewer.show(frame)

            key = cv2.waitKey(1) & 0xFF

            if key == ord("q"):
                break

            if key == ord("s") and not started:
                if not localization.valid():
                    print("No valid localization. Cannot start.")
                    continue

                if localization.landmark["id"] != 0:
                    print("Robot must be positioned at Landmark 0.")
                    continue

                # ESP32 auto-calibrates and zeros heading on boot.
                # Just set the speed and go.
                serial.set_speed(navigation.velocity * 1000.0)  # m/s → mm/s
                serial.drain_input()

                started = True
                last_current_landmark = localization.landmark["id"]
                print("Autonomous navigation started.")

    finally:
        serial.stop_move()
        serial.close()
        camera.release()
        viewer.close()


if __name__ == "__main__":
    main()
