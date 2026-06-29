from pupil_apriltags import Detector
import cv2

class AprilTagDetector:
    def __init__(self):
        self.detector = Detector(
            families="tag36h11", nthreads=4, quad_decimate=1.0, quad_sigma=0.0, refine_edges=True,
        )

    def detect(self, frame):
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        return self.detector.detect(gray)
    