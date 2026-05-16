import cv2
from cv2 import aruco
import numpy as np

WINDOW_NAME = "ArUco Detection"

# -----------------------------
# ArUco setup
# -----------------------------
aruco_dict = aruco.getPredefinedDictionary(
    aruco.DICT_4X4_50
)

detector_params = aruco.DetectorParameters()

detector = aruco.ArucoDetector(
    aruco_dict,
    detector_params
)

# -----------------------------
# Camera setup
# -----------------------------
cap = cv2.VideoCapture(0, cv2.CAP_DSHOW)

cap.set(cv2.CAP_PROP_FOURCC,cv2.VideoWriter_fourcc(*'MJPG'))
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)
cap.set(cv2.CAP_PROP_GAIN, 0)

# -----------------------------
# Window setup
# -----------------------------
cv2.namedWindow(WINDOW_NAME, cv2.WINDOW_NORMAL)
cv2.resizeWindow(WINDOW_NAME, 1280, 720)

while True:

    ret, frame = cap.read()

    if not ret:
        break

    # -----------------------------
    # Detect markers
    # -----------------------------
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

    corners, ids, rejected = detector.detectMarkers(gray)

    # -----------------------------
    # Draw detections
    # -----------------------------
    if ids is not None:

        aruco.drawDetectedMarkers(
            frame,
            corners,
            ids
        )

        # Print marker centers
        for i in range(len(ids)):

            pts = corners[i][0]

            cx = int(np.mean(pts[:, 0]))
            cy = int(np.mean(pts[:, 1]))

            marker_id = ids[i][0]

            cv2.circle(
                frame,
                (cx, cy),
                6,
                (0, 0, 255),
                -1
            )

            cv2.putText(
                frame,
                f"ID:{marker_id}",
                (cx + 10, cy),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.7,
                (0, 255, 0),
                2
            )

            print(f"Marker {marker_id}: ({cx}, {cy})")

    # -----------------------------
    # Show frame
    # -----------------------------
    cv2.imshow(WINDOW_NAME, frame)

    if cv2.getWindowProperty(
        WINDOW_NAME,
        cv2.WND_PROP_VISIBLE
    ) < 1:
        break

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()