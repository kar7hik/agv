import config
import camera
import detector

camera = camera.Camera()
detector = detector.AprilTagDetector()
camera.start()

while True:
    frame = camera.get_frame()
    detector.detect(frame)
