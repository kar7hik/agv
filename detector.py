from pupil_apriltags import Detector


class AprilTagDetector:
    def __init__(self):
        self.detector = Detector(
            families="tag36h11", nthreads=4, quad_decimate=1.0, quad_sigma=0.0
        )

    def detect(self, image):
        return self.detector.detect(image)
    