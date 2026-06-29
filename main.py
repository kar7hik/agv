import cv2

import config
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
                corners = detection.corners.astype(int)

                # Draw tag boundary
                for i in range(4):
                    p1 = tuple(corners[i])
                    p2 = tuple(corners[(i + 1) % 4])

                    cv2.line(frame, p1, p2, (0, 255, 0), 2)

                # Draw center
                center = tuple(detection.center.astype(int))
                cv2.circle(frame, center, 5, (0, 0, 255), -1)

                # Display Tag ID
                cv2.putText(
                    frame,
                    f"ID: {detection.tag_id}",
                    (corners[0][0], corners[0][1] - 10),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.6,
                    (255, 255, 0),
                    2,
                )

            cv2.imshow("AprilTag Detection", cv2.cvtColor(frame, cv2.COLOR_RGB2BGR))

            key = cv2.waitKey(1) & 0xFF

            if key == ord("q"):
                break

    finally:
        camera.release()
        cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
