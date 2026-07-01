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
    navigation.set_velocity(0.20)

    serial = SerialManager("/dev/ttyUSB0")

    camera.start()

    serial.ping()
    serial.enable()
    serial.zero_heading()

    try:
        while True:

            frame = camera.get_frame()

            detections = detector.detect(frame)

            geometry.update(detections)

            localization.update(detections)

            navigation.update(localization)

            if navigation.valid():

                velocity = navigation.velocity

                if navigation.current == navigation.target:
                    velocity = 0.0

                serial.send_velocity(
                    velocity,
                    navigation.desired_heading,
                    navigation.lateral_error,
                )

            viewer.draw(frame, detections)

            viewer.show(frame)

            key = cv2.waitKey(1) & 0xFF

            if key == ord("q"):
                break

    finally:

        serial.stop()
        serial.disable()
        serial.close()

        camera.release()
        viewer.close()


if __name__ == "__main__":
    main()
