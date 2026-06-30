import cv2

from camera import Camera
from detector import AprilTagDetector
from viewer import Viewer

import geometry
from landmark_map import LandmarkMap
fropm localization import Localization
from navigation import Navigation


def main():
    camera = Camera()
    detector = AprilTagDetector()
    viewer = Viewer()

    world = LandmarkMap("./maps/testbed.json")
    localization = Localization(world)
    navigation = Navigation()
    navigation.set_target(1)

    camera.start()

    try:
        while True:
            frame = camera.get_frame()
            detections = detector.detect(frame)
            geometry.update(detections)
            localization.update(detections)
            navigation.update(localization)
            viewer.draw(frame, detections)
            viewer.show(frame)

            key = cv2.waitKey(1) & 0xFF

            if key == ord("q"):
                break

    finally:
        camera.release()
        viewer.close()


if __name__ == "__main__":
    main()
