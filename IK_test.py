import math
import os

def solve_ik(x, y, z, phi, H, L1, L2, L3):
    # Base rotation

    L2offset = 12.5
    Hoffset = 13.92
    L2offset2 = 5.3
    L2offset3 = 17.05
    L2offset4 = 28.1
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
    L2a = L2 + L2offset4
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

    return (
        math.degrees(theta1),
        math.degrees(theta2),
        math.degrees(theta3),
        math.degrees(theta4),
    )

# Example
angles = solve_ik(
    x=59.62,
    y=177.5,
    z=272.75,
    phi=math.radians(69),     # end-effector pitch in radians
    H=97,      # base height
    L1=120,
    L2=89.75,
    L3=64.3
)

#os.system('cls' if os.name == 'nt' else 'clear')
print(angles)

