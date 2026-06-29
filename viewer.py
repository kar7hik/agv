import cv2
import geometry


class Viewer:
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

            if detection.yaw is None:
                yaw_text = "Yaw: N/A"
            else:
                yaw_text = f"Yaw: {detection.yaw:.1f} deg"

            cv2.putText(
                frame,
                yaw_text,
                (corners[0][0], corners[0][1] + 20),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.55,
                (0, 255, 255),
                2,
            )

            cv2.putText(
                frame,
                f"X : {detection.x_offset:.3f} m",
                (corners[0][0], corners[0][1] + 40),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.55,
                (255, 0, 255),
                2,
            )
        return frame

    def show(self, frame):
        cv2.imshow(self.window_name, cv2.cvtColor(frame, cv2.COLOR_RGB2BGR))

    def close(self):
        cv2.destroyAllWindows()
