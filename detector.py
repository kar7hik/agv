from pupil_apriltags import Detector
import cv2
import config


class AprilTagDetector:
    def __init__(self):
        self.detector = Detector(
            families="tag36h11",
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
            camera_params=config.CAMERA_PARAMS,
            tag_size=config.TAG_SIZE,
        )
