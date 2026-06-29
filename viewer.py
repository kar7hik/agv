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
                0.6,
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
                (x, y + 15),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.55,
                (0, 125, 125),
                2,
            )

            # Draw Lateral Offset
            if detection.lateral is None:
                lateral_text = "Yaw: N/A"
            else:
                lateral_text = f"Yaw: {detection.lateral:.3f} m"

            cv2.putText(
                frame,
                lateral_text,
                (x, y + 30),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.55,
                (0, 255, 255),
                2,
            )

            # Draw Distance
            if detection.distance is None:
                distance_text = "Distance: N/A"
            else:
                distance_text = f"Distance: {detection.distance:.3f} m"

            cv2.putText(
                frame,
                distance_text,
                (x, y + 45),
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
