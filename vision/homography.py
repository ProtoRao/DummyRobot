import cv2
from cv2 import aruco
import numpy as np
import math

# -----------------------------------
# Globals
# -----------------------------------

clicked_pixel = None
H = None

# -----------------------------------
# Mouse callback
# -----------------------------------
Z_FIXED = 150

from servo_position_gui import DEFAULT_PORT, PositionalServoBridgeClient
client = PositionalServoBridgeClient(DEFAULT_PORT)
client.connect()

def move_robot_to_xyz(
    x: float,
    y: float,
    z: float,
) -> list[float]:
    # Base rotation
    L1 = 120.0
    L2 = 89.75
    L3 = 64.3
    H = 97.0
    phi = 0.0
    Hoffset = 13.92
    L2offset = 28.1
    L3offset = 7.95
    L3offset2 = 11.5

    theta1 = math.atan2(y, x) - math.asin(L3offset / math.sqrt(x*x + y*y))

    # Planar distance
    x = x + L3offset * math.sin(theta1)
    y = y - L3offset * math.cos(theta1)

    r = math.sqrt(x*x + y*y) - Hoffset

    # Shift into shoulder frame
    zs = z - H

    # Wrist center
    rw = r - L3 * math.cos(phi) + L3offset2 * math.sin(phi)
    zw = zs - L3 * math.sin(phi) - L3offset2 * math.cos(phi)
                                                      
    # Elbow IK
    L2a = L2 + L2offset
    D = (rw**2 + zw**2 - L1**2 - L2a**2) / (2 * L1 * L2a)

    # Reachability check
    if abs(D) > 1:
        return None

    # Elbow-down solution
    theta3 = math.atan2(math.sqrt(1 - D*D), D)

    # Shoulder angle
    theta2 = (
        math.atan2(zw, rw)
        + math.atan2(
            L2a * math.sin(theta3),
            L1 + L2a * math.cos(theta3)
        )
    )

    # Wrist angle
    theta4 = phi + theta3 - theta2

    theta3 = theta3 * -1  # Invert theta3 to match the physical configuration of the robot

    print(f"Calculated angles (degrees): {[math.degrees(theta1), math.degrees(theta2), math.degrees(theta3), math.degrees(theta4)]}")
    servo_angles_deg = [math.degrees(theta1), math.degrees(theta2), math.degrees(theta3), 0, math.degrees(theta4)]
    try:
        for i in range(5): client.set_angle(i, servo_angles_deg[i])
    except Exception as e: print("Move Failed", str(e))

    return

def mouse_callback(event, x, y, flags, param):

    global clicked_pixel
    global H

    if event == cv2.EVENT_LBUTTONDOWN:

        clicked_pixel = (x, y)

        print(f"\nClicked pixel: {clicked_pixel}")

        # -----------------------------------
        # Convert pixel -> world coordinates
        # -----------------------------------

        if H is not None:

            px = np.array(
                [[[x, y]]],
                dtype=np.float32
            )

            world_pt = cv2.perspectiveTransform(
                px,
                H
            )

            X = world_pt[0][0][0]
            Y = world_pt[0][0][1]

            print(
                f"World Coordinates:"
                f" ({X:.1f}, {Y:.1f}) mm"
            )

            # -----------------------------------
            # Call robot function
            # -----------------------------------

            move_robot_to_xyz(
                X,
                Y,
                Z_FIXED
            )

aruco_dict = aruco.getPredefinedDictionary(
    aruco.DICT_4X4_50
)
detector = aruco.ArucoDetector(aruco_dict)

# -----------------------------------
# Known world coordinates in mm
# -----------------------------------

world_points_dict = {
    3: [-230, 80],
    0: [-230, 320],
    1: [230, 320],
    2: [230, 80]
}

cap = cv2.VideoCapture(0, cv2.CAP_DSHOW)
cap.set(cv2.CAP_PROP_FOURCC,cv2.VideoWriter_fourcc(*'MJPG'))
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)
cap.set(cv2.CAP_PROP_GAIN, 0)

WINDOW_NAME = "Calibration"

cv2.namedWindow(WINDOW_NAME)

cv2.setMouseCallback(
    WINDOW_NAME,
    mouse_callback
)

# -----------------------------------
# Main loop
# -----------------------------------

while True:

    ret, frame = cap.read()

    if not ret:
        break

    gray = cv2.cvtColor(
        frame,
        cv2.COLOR_BGR2GRAY
    )

    corners, ids, _ = detector.detectMarkers(gray)

    # -----------------------------------
    # Compute homography
    # -----------------------------------

    if ids is not None and len(ids) >= 4:

        ids = ids.flatten()

        image_points = []
        world_points = []

        for i, marker_id in enumerate(ids):

            if marker_id in world_points_dict:

                pts = corners[i][0]

                cx = np.mean(pts[:, 0])
                cy = np.mean(pts[:, 1])

                image_points.append([cx, cy])

                world_points.append(
                    world_points_dict[marker_id]
                )

                cv2.circle(
                    frame,
                    (int(cx), int(cy)),
                    6,
                    (0,0,255),
                    -1
                )

                aruco.drawDetectedMarkers(
                    frame,
                    corners,
                    ids.reshape(-1,1)
                )

        image_points = np.array(
            image_points,
            dtype=np.float32
        )

        world_points = np.array(
            world_points,
            dtype=np.float32
        )

        H, _ = cv2.findHomography(
            image_points,
            world_points
        )

    # -----------------------------------
    # Draw clicked point
    # -----------------------------------

    if clicked_pixel is not None:

        cv2.circle(
            frame,
            clicked_pixel,
            8,
            (255,0,0),
            -1
        )

    # -----------------------------------
    # Show frame
    # -----------------------------------

    cv2.imshow(WINDOW_NAME, frame)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()