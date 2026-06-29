import config
import cv2
from camera import Camera
from detector import AprilTagDetector


def main():
    camera = Camera()
    detector = AprilTagDetector()
    
    camera.start()
    
    try:
        while True:
            frame = camera.get_frame()
            detections = detector.detect(frame)

            for detection in detections:
                cv2.circle(frame, detection.corners[0], 5, (0, 0, 255), 2)
                cv2.circle(frame, detection.corners[1], 5, (0, 0, 255), 2)
                cv2.circle(frame, detection.corners[2], 5, (0, 0, 255), 2)
                cv2.circle(frame, detection.corners[3], 5, (0, 0, 255), 2)

            cv2.imshow("frame", cv2.cvtColor(frame, cv2.COLOR_BGR2RGB))
            key = cv2.waitKey(1)
            
            if key == ord("q"):
                break
            
    finally:
        camera.release()
        cv2.destroyAllWindows()
        
if 