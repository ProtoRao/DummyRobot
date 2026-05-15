import math
import os

def solve_ik(x, y, z, phi, H, L1, L2, L3):
    # Base rotation

    L2offset = 12.5
    Hoffset = 13.92
    L2offset2 = 5.3

    theta1 = math.atan2(y, x) - math.asin(L2offset / math.sqrt(x*x + y*y))

    # Planar distance
    x = x + L2offset * math.sin(theta1)
    y = y - L2offset * math.cos(theta1)

    r = math.sqrt(x*x + y*y) - Hoffset

    # Shift into shoulder frame
    zs = z - H

    # Wrist center
    rw = r #- L3 * math.cos(phi)
    zw = zs #- L3 * math.sin(phi)
                                                      
    # Elbow IK
    L2a = math.sqrt (L2**2 + L2offset2**2) # Adjust L2 for the offset
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

    theta3 = math.atan2(L2offset2, L2) - theta3  # Adjust theta3 for the offset
    theta3 = theta3 * -1
    # Wrist angle
    theta4 = phi - theta2 - theta3

    return (
        math.degrees(theta1),
        math.degrees(theta2),
        math.degrees(theta3),
        math.degrees(theta4),
    )

# Example
angles = solve_ik(
    x=149.71,
    y=100.869,
    z=59.423,
    phi=0,     # end-effector pitch in radians
    H=97,      # base height
    L1=120,
    L2=89.75, #+28.1, # upper arm + forearm offset
    L3=0
)

#os.system('cls' if os.name == 'nt' else 'clear')
print(angles)

