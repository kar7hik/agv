import config
import camera
import detector

camera = Camera()
detector = AprilTagDetector()
camera.start()

while True:
    frame = camera.get_frame()
    detector.detect(frame)
