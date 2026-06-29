import cv2

import config
from camera import Camera
from detector import AprilTagDetector
from visualization import 


def main():
    camera = Camera()
    detector = AprilTagDetector()
    visualization = Visulaization()
    camera.start()

    try:
        while True:
            frame = camera.get_frame()
            detections = detector.detect(frame)
            frame = visualization.draw(frame, detections)
            visualization.show(frame)
            
            key = cv2.waitKey(1) & 0xFF

            if key == ord("q"):
                break

    finally:
        camera.release()
        visualization.close()


if __name__ == "__main__":
    main()
