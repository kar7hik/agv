import cv2

import config
from camera import Camera
from detector import AprilTagDetector
from viewer import Viewer


def main():
    camera = Camera()
    detector = AprilTagDetector()
    viewer = Viewer()
    camera.start()

    try:
        while True:
            frame = camera.get_frame()
            detections = detector.detect(frame)
            frame = viewer.draw(frame, detections)
            viewer.show(frame)
            
            key = cv2.waitKey(1) & 0xFF

            if key == ord("q"):
                break

    finally:
        camera.release()
        viewer.close()


if __name__ == "__main__":
    main()
