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

            # Draw Center Circle
            center = tuple(detection.center.astype(int))
            x = corners[0][0]
            y = corners[0][1]
            cv2.circle(frame, center, 5, (0, 0, 255), -1)

            # Draw TAG ID
            cv2.putText(
                frame,
                f"ID: {detection.tag_id}",
                (x, y - 10),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.5,
                (0, 255, 0),
                2,
            )

            # Draw Heading
            if detection.heading is None:
                heading_text = "Heading: N/A"
            else:
                heading_text = f"Heading: {detection.heading:.1f} deg"

            cv2.putText(
                frame,
                heading_text,
                (x, y + 20),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.5,
                (255, 255, 255),
                2,
            )

            # Draw Lateral Offset
            if detection.lateral is None:
                lateral_text = "Lateral: N/A"
            else:
                lateral_text = f"Lateral: {detection.lateral:.3f} m"

            cv2.putText(
                frame,
                lateral_text,
                (x, y + 40),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.5,
                (0, 255, 255),
                2,
            )

            # Draw Forward Offset
            if detection.forward is None:
                forward_text = "Forward: N/A"
            else:
                forward_text = f"Forward: {detection.forward:.3f} m"

            cv2.putText(
                frame,
                forward_text,
                (x, y + 60),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.5,
                (255, 0, 255),
                2,
            )

        return frame

    def show(self, frame):
        cv2.imshow(self.window_name, cv2.cvtColor(frame, cv2.COLOR_RGB2BGR))

    def close(self):
        cv2.destroyAllWindows()
