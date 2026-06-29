import cv2

class Visualization:
    def __init__(self):
        self.window_name = "AprilTag Detection"
        
    def draw(self, frame, detections):
        for detection in detections:
            corners = detection.corners.astype(int)
            
            for i in range(4):
                p1 = tuple(corners[i])
                p2 = tuple(corners[(i + 1) % 4])
                
                cv2.line(frame, p1, p2, (0, 255, 0), 2)
                center = tuple(detection.center.astype(int))
                cv2.circle(frame, center, 5, (0, 0, 255), -1)
                
                cv2.putText(
                    frame,
                    f"ID: {detection.tag_id}",
                    (corners[0][0], corners[0][1] - 10),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.5,
                    (0, 255, 0),
                    2,
                )
        return frame
    
    def show(self, frame):
        cv2.imshow(self.window_name, cv2.cvtColor(frame, cv2.COLOR_BGR2RGB))
        
    def close(self):
        cv2.destroyAllWindows()



