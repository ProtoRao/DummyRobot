"""
Highly simplified version of ik_position_gui.py.

This file intentionally removes:
- Tkinter GUI
- serial / Arduino communication
- classes
- dataclasses
- OOP-style app structure

It keeps only the core ideas:
- servo angles are converted into model angles
- forward kinematics turns joint angles into XYZ
- inverse kinematics searches for joint angles that produce a target XYZ
- the Jacobian is estimated numerically with small angle nudges

Run:
    python ik_position_simplified.py
"""

import math


# ---------------------------------------------------------------------------
# 1. Robot geometry and calibration
# ---------------------------------------------------------------------------

# Physical measurements in millimeters.
D1_MM = 96.88
D2_MM = 4.2
A2_MM = 120.0
A3_MM = 94.75

# The IK code controls only the first 3 servos.
# The real robot has 5, but servos 4 and 5 are ignored here.
INITIAL_SERVO_ANGLES = [90, 90, 0, 0, 0]

# Mechanical safety limits for each servo.
ANGLE_LIMITS = [
    (0, 180),
    (0, 180),
    (-90, 90),
    (-90, 90),
    (-90, 90),
]

# Difference between real servo angles and mathematical model angles.
#
# model_angle = servo_angle - offset
# servo_angle = model_angle + offset
#
# Example:
#   servo [90, 90, 0] becomes model [90, 30, 15]
SERVO_MODEL_OFFSETS = [0.0, 60.0, -15.0, 0.0, 0.0]

# Servo limits converted into model-angle limits.
# During IK, we clamp floating-point model angles to these limits.
# We should not round back into integer servo angles until the end.
MODEL_LIMITS = []
for i in range(3):
    low, high = ANGLE_LIMITS[i]
    MODEL_LIMITS.append(
        (
            low - SERVO_MODEL_OFFSETS[i],
            high - SERVO_MODEL_OFFSETS[i],
        )
    )

# IK accepts a solution if the final XYZ position is within this distance.
POSITION_TOLERANCE_MM = 2.0


# ---------------------------------------------------------------------------
# 2. Small utilities
# ---------------------------------------------------------------------------

def clamp(value, low, high):
    return max(low, min(high, value))


def servo_to_model_degrees(servo_angles):
    model_angles = []
    for i in range(len(servo_angles)):
        model_angles.append(servo_angles[i] - SERVO_MODEL_OFFSETS[i])
    return model_angles


def model_to_servo_degrees(model_angles):
    servo_angles = []
    for i in range(len(model_angles)):
        servo_angle = round(model_angles[i] + SERVO_MODEL_OFFSETS[i])
        low, high = ANGLE_LIMITS[i]
        servo_angles.append(int(clamp(servo_angle, low, high)))
    return servo_angles


def clamp_model_angles(model_angles):
    clamped = model_angles[:]
    for i in range(3):
        low, high = MODEL_LIMITS[i]
        clamped[i] = clamp(clamped[i], low, high)
    return clamped


# ---------------------------------------------------------------------------
# 3. Matrix math for forward kinematics
# ---------------------------------------------------------------------------

def matrix_multiply(a, b):
    result = []
    for row in range(len(a)):
        result_row = []
        for col in range(len(b[0])):
            total = 0.0
            for k in range(len(b)):
                total += a[row][k] * b[k][col]
            result_row.append(total)
        result.append(result_row)
    return result


def dh_transform(theta_rad, d_mm, a_mm, alpha_rad):
    """
    Create one Denavit-Hartenberg transform matrix.

    A transform matrix stores both:
    - rotation
    - translation

    The final XYZ position is stored in column 4:
        matrix[0][3] -> x
        matrix[1][3] -> y
        matrix[2][3] -> z
    """
    ct = math.cos(theta_rad)
    st = math.sin(theta_rad)
    ca = math.cos(alpha_rad)
    sa = math.sin(alpha_rad)

    return [
        [ct, -st * ca, st * sa, a_mm * ct],
        [st, ct * ca, -ct * sa, a_mm * st],
        [0.0, sa, ca, d_mm],
        [0.0, 0.0, 0.0, 1.0],
    ]


def forward_kinematics(model_angles_deg):
    """
    Forward kinematics:

        joint angles -> XYZ position

    Input:
        model_angles_deg = [base, shoulder, elbow]

    Output:
        [x, y, z]
    """
    base = math.radians(model_angles_deg[0])
    shoulder = math.radians(model_angles_deg[1])
    elbow = math.radians(model_angles_deg[2])

    # Identity matrix: "no movement yet".
    transform = [
        [1.0, 0.0, 0.0, 0.0],
        [0.0, 1.0, 0.0, 0.0],
        [0.0, 0.0, 1.0, 0.0],
        [0.0, 0.0, 0.0, 1.0],
    ]

    # These are the same 3 transforms used by ik_position_gui.py.
    transforms = [
        dh_transform(base, D1_MM, 0.0, math.pi / 2.0),
        dh_transform(shoulder, D2_MM, A2_MM, 0.0),
        dh_transform(elbow, 0.0, A3_MM / 2.0, 0.0),
    ]

    # Apply the transforms in order: base -> shoulder -> elbow.
    for next_transform in transforms:
        transform = matrix_multiply(transform, next_transform)

    x = transform[0][3]
    y = transform[1][3]
    z = transform[2][3]
    return [x, y, z]


# ---------------------------------------------------------------------------
# 4. Error and Jacobian
# ---------------------------------------------------------------------------

def position_error(current_xyz, target_xyz):
    """
    Difference between where the robot is and where we want it to be.

    Positive error means:
        current position - target position
    """
    return [
        current_xyz[0] - target_xyz[0],
        current_xyz[1] - target_xyz[1],
        current_xyz[2] - target_xyz[2],
    ]


def error_length(error_xyz):
    return math.sqrt(
        error_xyz[0] ** 2
        + error_xyz[1] ** 2
        + error_xyz[2] ** 2
    )


def estimate_jacobian(model_angles_deg, target_xyz):
    """
    Estimate the Jacobian numerically.

    The Jacobian answers:

        If I slightly change joint 1, 2, or 3,
        how does the XYZ error change?

    This code does not derive formulas manually.
    It nudges each joint by 1 degree and observes the effect.
    """
    current_xyz = forward_kinematics(model_angles_deg)
    current_error = position_error(current_xyz, target_xyz)

    jacobian = [
        [0.0, 0.0, 0.0],
        [0.0, 0.0, 0.0],
        [0.0, 0.0, 0.0],
    ]

    nudge_deg = 1.0
    nudge_rad = math.radians(nudge_deg)

    for joint in range(3):
        nudged_angles = model_angles_deg[:]
        nudged_angles[joint] += nudge_deg

        nudged_xyz = forward_kinematics(nudged_angles)
        nudged_error = position_error(nudged_xyz, target_xyz)

        # Each column says how XYZ error changes when this joint changes.
        for axis in range(3):
            jacobian[axis][joint] = (
                nudged_error[axis] - current_error[axis]
            ) / nudge_rad

    return jacobian


# ---------------------------------------------------------------------------
# 5. Tiny 3x3 linear solver
# ---------------------------------------------------------------------------

def solve_3x3(matrix, vector):
    """
    Solve:

        matrix * x = vector

    Used by IK to find the joint-angle correction.

    Returns:
        [delta_joint_1, delta_joint_2, delta_joint_3]

    or None if the matrix is too close to singular.
    """
    augmented = []
    for row, value in zip(matrix, vector):
        augmented.append(row[:] + [value])

    size = 3

    for pivot_index in range(size):
        pivot_row = max(
            range(pivot_index, size),
            key=lambda row: abs(augmented[row][pivot_index]),
        )

        if abs(augmented[pivot_row][pivot_index]) < 1e-9:
            return None

        augmented[pivot_index], augmented[pivot_row] = (
            augmented[pivot_row],
            augmented[pivot_index],
        )

        pivot_value = augmented[pivot_index][pivot_index]
        for col in range(pivot_index, size + 1):
            augmented[pivot_index][col] /= pivot_value

        for row in range(size):
            if row == pivot_index:
                continue

            factor = augmented[row][pivot_index]
            for col in range(pivot_index, size + 1):
                augmented[row][col] -= factor * augmented[pivot_index][col]

    return [
        augmented[0][3],
        augmented[1][3],
        augmented[2][3],
    ]


# ---------------------------------------------------------------------------
# 6. Inverse kinematics
# ---------------------------------------------------------------------------

def improve_guess_once(model_angles_deg, target_xyz):
    """
    Do one IK improvement step.

    This uses a damped least-squares style update:

        J = Jacobian
        error = current_xyz - target_xyz

        Solve:
            (J^T J + damping) * delta = -J^T error

        Then:
            angles = angles + delta

    Damping helps when the math gets unstable near awkward poses.
    """
    current_xyz = forward_kinematics(model_angles_deg)
    error = position_error(current_xyz, target_xyz)
    jacobian = estimate_jacobian(model_angles_deg, target_xyz)

    damping = 0.15

    # Build J^T J.
    normal_matrix = [
        [0.0, 0.0, 0.0],
        [0.0, 0.0, 0.0],
        [0.0, 0.0, 0.0],
    ]

    for row in range(3):
        for col in range(3):
            total = 0.0
            for axis in range(3):
                total += jacobian[axis][row] * jacobian[axis][col]
            normal_matrix[row][col] = total

        # Add damping to the diagonal.
        normal_matrix[row][row] += damping

    # Build -J^T error.
    normal_vector = [0.0, 0.0, 0.0]
    for row in range(3):
        total = 0.0
        for axis in range(3):
            total += jacobian[axis][row] * error[axis]
        normal_vector[row] = -total

    delta_rad = solve_3x3(normal_matrix, normal_vector)
    if delta_rad is None:
        return None

    improved_angles = model_angles_deg[:]
    for joint in range(3):
        improved_angles[joint] += math.degrees(delta_rad[joint])

    # Clamp model angles using the equivalent servo limits.
    #
    # Important:
    # Do not round to integer servo angles inside the IK loop.
    # Rounding every iteration can make the solver bounce around and fail.
    improved_angles = clamp_model_angles(improved_angles)

    return improved_angles


def inverse_kinematics(target_xyz, current_servo_angles):
    """
    Inverse kinematics:

        target XYZ -> servo angles

    This simplified version tries only one starting guess:
    the current robot position.

    The real ik_position_gui.py tries many starting guesses because one target
    can have multiple valid arm postures.
    """
    model_angles = servo_to_model_degrees(current_servo_angles)[:3]

    for iteration in range(80):
        current_xyz = forward_kinematics(model_angles)
        error = position_error(current_xyz, target_xyz)
        distance = error_length(error)

        print(
            "iteration",
            iteration,
            "model",
            [round(a, 2) for a in model_angles],
            "xyz",
            [round(v, 2) for v in current_xyz],
            "error_mm",
            round(distance, 2),
        )

        if distance <= POSITION_TOLERANCE_MM:
            servo_angles = model_to_servo_degrees(model_angles + [0.0, 0.0])
            return servo_angles[:3], current_xyz

        improved = improve_guess_once(model_angles, target_xyz)
        if improved is None:
            raise RuntimeError("IK failed because the Jacobian became singular.")

        model_angles = improved

    raise RuntimeError("IK failed because it did not converge within 80 iterations.")


# ---------------------------------------------------------------------------
# 7. Example information flow
# ---------------------------------------------------------------------------

def main():
    current_servo_angles = INITIAL_SERVO_ANGLES[:]

    print("Current servo angles:", current_servo_angles[:3])

    current_model_angles = servo_to_model_degrees(current_servo_angles)
    print("Current model angles:", current_model_angles[:3])

    current_xyz = forward_kinematics(current_model_angles)
    print("Current XYZ:", [round(v, 2) for v in current_xyz])

    # Pick a reachable teaching target by first choosing some model angles,
    # then using forward kinematics to get the XYZ position.
    #
    # This makes the example easier to understand:
    #     known angles -> known XYZ -> ask IK to recover angles
    #
    # The full GUI can handle more targets because it tries many starting
    # guesses. This simplified file intentionally starts only from the current
    # robot pose.
    target_xyz = forward_kinematics([90.0, 25.0, 30.0])
    print("Target XYZ:", target_xyz)

    solved_servo_angles, solved_xyz = inverse_kinematics(
        target_xyz,
        current_servo_angles,
    )

    print("Solved servo angles:", solved_servo_angles)
    print("Solved XYZ:", [round(v, 2) for v in solved_xyz])
    print("Final error:", round(error_length(position_error(solved_xyz, target_xyz)), 2), "mm")


if __name__ == "__main__":
    main()
