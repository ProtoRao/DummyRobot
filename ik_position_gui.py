import math
import tkinter as tk
from dataclasses import dataclass
from tkinter import messagebox

from servo_position_gui import ANGLE_LIMITS, DEFAULT_PORT, INITIAL_ANGLES, PositionalServoBridgeClient


D1_MM = 96.88
D2_MM = 4.2
A2_MM = 120.0
A3_MM = 94.75
POSITION_STEP_MM = 10.0
POSITION_TOLERANCE_MM = 2.0
SINGULARITY_EPSILON = 1e-6

# Calibrated model mapping chosen so servo [90, 90, 0] aligns with the photographed
# 3-link reference posture when XYZ is measured at the midpoint of link 3.
SERVO_MODEL_OFFSETS = [0.0, 60.0, -15.0, 0.0, 0.0]
MODEL_LIMITS_DEG = [
    (ANGLE_LIMITS[0][0] - SERVO_MODEL_OFFSETS[0], ANGLE_LIMITS[0][1] - SERVO_MODEL_OFFSETS[0]),
    (ANGLE_LIMITS[1][0] - SERVO_MODEL_OFFSETS[1], ANGLE_LIMITS[1][1] - SERVO_MODEL_OFFSETS[1]),
    (ANGLE_LIMITS[2][0] - SERVO_MODEL_OFFSETS[2], ANGLE_LIMITS[2][1] - SERVO_MODEL_OFFSETS[2]),
]


@dataclass
class CartesianPose:
    x_mm: float
    y_mm: float
    z_mm: float


@dataclass
class IKSolution:
    model_angles_deg: list[float]
    servo_angles_deg: list[int]
    pose: CartesianPose
    position_error_mm: float
    branch_label: str
    total_joint_delta_deg: float


def clamp(value: float, low: float, high: float) -> float:
    return max(low, min(high, value))


def servo_to_model_degrees(servo_angles_deg: list[float]) -> list[float]:
    return [servo_angles_deg[index] - SERVO_MODEL_OFFSETS[index] for index in range(len(servo_angles_deg))]


def model_to_servo_degrees(model_angles_deg: list[float]) -> list[int]:
    servo_angles: list[int] = []
    for index, model_angle in enumerate(model_angles_deg):
        servo_angle = int(round(model_angle + SERVO_MODEL_OFFSETS[index]))
        min_angle, max_angle = ANGLE_LIMITS[index]
        servo_angles.append(int(clamp(servo_angle, min_angle, max_angle)))
    return servo_angles


def matrix_multiply(left: list[list[float]], right: list[list[float]]) -> list[list[float]]:
    return [
        [sum(left[row][k] * right[k][column] for k in range(len(right))) for column in range(len(right[0]))]
        for row in range(len(left))
    ]


def dh_transform(theta_rad: float, d_mm: float, a_mm: float, alpha_rad: float) -> list[list[float]]:
    cos_theta = math.cos(theta_rad)
    sin_theta = math.sin(theta_rad)
    cos_alpha = math.cos(alpha_rad)
    sin_alpha = math.sin(alpha_rad)
    return [
        [cos_theta, -sin_theta * cos_alpha, sin_theta * sin_alpha, a_mm * cos_theta],
        [sin_theta, cos_theta * cos_alpha, -cos_theta * sin_alpha, a_mm * sin_theta],
        [0.0, sin_alpha, cos_alpha, d_mm],
        [0.0, 0.0, 0.0, 1.0],
    ]


def compose_reference_transform(model_angles_rad: list[float]) -> list[list[float]]:
    transforms = [
        dh_transform(model_angles_rad[0], D1_MM, 0.0, math.pi / 2.0),
        dh_transform(model_angles_rad[1], D2_MM, A2_MM, 0.0),
        dh_transform(model_angles_rad[2], 0.0, A3_MM / 2.0, 0.0),
    ]

    result = [
        [1.0, 0.0, 0.0, 0.0],
        [0.0, 1.0, 0.0, 0.0],
        [0.0, 0.0, 1.0, 0.0],
        [0.0, 0.0, 0.0, 1.0],
    ]
    for transform in transforms:
        result = matrix_multiply(result, transform)
    return result


def forward_kinematics_from_model(model_angles_deg: list[float]) -> CartesianPose:
    transform = compose_reference_transform([math.radians(value) for value in model_angles_deg[:3]])
    return CartesianPose(
        x_mm=transform[0][3],
        y_mm=transform[1][3],
        z_mm=transform[2][3],
    )


def solve_linear_system_3x3(matrix: list[list[float]], vector: list[float]) -> list[float] | None:
    augmented = [row[:] + [value] for row, value in zip(matrix, vector)]
    size = 3

    for pivot_index in range(size):
        pivot_row = max(range(pivot_index, size), key=lambda row_index: abs(augmented[row_index][pivot_index]))
        if abs(augmented[pivot_row][pivot_index]) < 1e-9:
            return None
        augmented[pivot_index], augmented[pivot_row] = augmented[pivot_row], augmented[pivot_index]

        pivot_value = augmented[pivot_index][pivot_index]
        for column_index in range(pivot_index, size + 1):
            augmented[pivot_index][column_index] /= pivot_value

        for row_index in range(size):
            if row_index == pivot_index:
                continue
            factor = augmented[row_index][pivot_index]
            for column_index in range(pivot_index, size + 1):
                augmented[row_index][column_index] -= factor * augmented[pivot_index][column_index]

    return [augmented[row_index][size] for row_index in range(size)]


def pose_error_vector(pose: CartesianPose, target_x_mm: float, target_y_mm: float, target_z_mm: float) -> list[float]:
    return [pose.x_mm - target_x_mm, pose.y_mm - target_y_mm, pose.z_mm - target_z_mm]


def refine_solution(seed_model_deg: list[float], target_x_mm: float, target_y_mm: float, target_z_mm: float) -> list[float] | None:
    model_angles_deg = seed_model_deg[:3]
    damping = 0.15

    for _ in range(80):
        pose = forward_kinematics_from_model(model_angles_deg)
        error_vector = pose_error_vector(pose, target_x_mm, target_y_mm, target_z_mm)
        position_error_mm = math.sqrt(sum(component * component for component in error_vector))
        if position_error_mm <= POSITION_TOLERANCE_MM:
            return model_angles_deg

        jacobian = [[0.0 for _ in range(3)] for _ in range(3)]
        finite_delta_deg = 1.0
        for joint_index in range(3):
            shifted = model_angles_deg[:]
            shifted[joint_index] += finite_delta_deg
            shifted_pose = forward_kinematics_from_model(shifted)
            shifted_error = pose_error_vector(shifted_pose, target_x_mm, target_y_mm, target_z_mm)
            for row_index in range(3):
                jacobian[row_index][joint_index] = (shifted_error[row_index] - error_vector[row_index]) / math.radians(
                    finite_delta_deg
                )

        normal_matrix = [[0.0 for _ in range(3)] for _ in range(3)]
        normal_vector = [0.0 for _ in range(3)]
        for row_index in range(3):
            for column_index in range(3):
                normal_matrix[row_index][column_index] = sum(
                    jacobian[k][row_index] * jacobian[k][column_index] for k in range(3)
                )
            normal_matrix[row_index][row_index] += damping
            normal_vector[row_index] = -sum(jacobian[k][row_index] * error_vector[k] for k in range(3))

        delta = solve_linear_system_3x3(normal_matrix, normal_vector)
        if delta is None:
            return None

        for joint_index in range(3):
            model_angles_deg[joint_index] += math.degrees(delta[joint_index])
            min_limit_deg, max_limit_deg = MODEL_LIMITS_DEG[joint_index]
            model_angles_deg[joint_index] = clamp(model_angles_deg[joint_index], min_limit_deg, max_limit_deg)

    return None


def seed_candidate_angles(target_x_mm: float, target_y_mm: float, current_model_deg: list[float]) -> list[tuple[str, list[float]]]:
    if math.hypot(target_x_mm, target_y_mm) < SINGULARITY_EPSILON:
        raise ValueError("Target is on the base axis; the 3-link base rotation is undefined there.")

    base_guess_deg = math.degrees(math.atan2(target_y_mm, target_x_mm))
    base_candidates_deg = [base_guess_deg, base_guess_deg + 180.0, current_model_deg[0], 90.0, -90.0]

    shoulder_candidates_deg = [
        current_model_deg[1],
        30.0,
        15.0,
        0.0,
        45.0,
        60.0,
        -15.0,
    ]
    elbow_candidates_deg = [
        current_model_deg[2],
        15.0,
        0.0,
        -15.0,
        30.0,
        -30.0,
        45.0,
    ]

    seeds: list[tuple[str, list[float]]] = [("current-state", current_model_deg[:3])]
    seen_servo_states: set[tuple[int, int, int]] = set()

    for base_deg in base_candidates_deg:
        for shoulder_deg in shoulder_candidates_deg:
            for elbow_deg in elbow_candidates_deg:
                seed = [base_deg, shoulder_deg, elbow_deg]
                servo_guess = tuple(model_to_servo_degrees(seed + [0.0, 0.0])[:3])
                if servo_guess in seen_servo_states:
                    continue
                seen_servo_states.add(servo_guess)
                seeds.append((f"grid/{base_deg:.1f}/{shoulder_deg:.1f}/{elbow_deg:.1f}", seed))

    return seeds


def choose_best_solution(solutions: list[IKSolution]) -> IKSolution:
    return min(
        solutions,
        key=lambda solution: (
            round(solution.total_joint_delta_deg, 6),
            round(solution.position_error_mm, 6),
        ),
    )


def solve_xyz_inverse_kinematics(
    target_x_mm: float,
    target_y_mm: float,
    target_z_mm: float,
    current_servo_deg: list[float],
) -> tuple[IKSolution, list[IKSolution]]:
    current_model_deg = servo_to_model_degrees(current_servo_deg)
    seeds = seed_candidate_angles(target_x_mm, target_y_mm, current_model_deg)

    valid_solutions: list[IKSolution] = []
    for branch_label, seed in seeds:
        refined = refine_solution(seed, target_x_mm, target_y_mm, target_z_mm)
        if refined is None:
            continue

        servo_angles_deg = model_to_servo_degrees(refined + current_model_deg[3:])
        pose = forward_kinematics_from_model(refined)
        position_error_mm = math.dist(
            [pose.x_mm, pose.y_mm, pose.z_mm],
            [target_x_mm, target_y_mm, target_z_mm],
        )
        if position_error_mm > POSITION_TOLERANCE_MM:
            continue

        total_joint_delta_deg = sum(abs(refined[index] - current_model_deg[index]) for index in range(3))
        valid_solutions.append(
            IKSolution(
                model_angles_deg=refined + current_model_deg[3:],
                servo_angles_deg=servo_angles_deg,
                pose=pose,
                position_error_mm=position_error_mm,
                branch_label=branch_label,
                total_joint_delta_deg=total_joint_delta_deg,
            )
        )

    if not valid_solutions:
        raise ValueError("No valid 3-link XYZ solution met the current joint-limit and workspace checks.")

    return choose_best_solution(valid_solutions), valid_solutions


class CartesianIKApp:
    def __init__(self, root: tk.Tk) -> None:
        self.root = root
        self.root.title("3-Link Cartesian XYZ Controller")
        self.client = PositionalServoBridgeClient(DEFAULT_PORT)
        self.preview_solution: IKSolution | None = None
        self.current_servo_angles = INITIAL_ANGLES[:]
        self.current_pose: CartesianPose | None = None

        self.status_var = tk.StringVar(value="Disconnected")
        self.target_vars = {
            "x": tk.StringVar(value="4.2"),
            "y": tk.StringVar(value="137.4"),
            "z": tk.StringVar(value="190.4"),
        }
        self.pose_vars = {
            "x": tk.StringVar(value="--"),
            "y": tk.StringVar(value="--"),
            "z": tk.StringVar(value="--"),
        }
        self.solution_rows: list[dict[str, tk.StringVar]] = []

        self._build_ui()
        self._connect_and_refresh()
        self.root.protocol("WM_DELETE_WINDOW", self.on_close)

    def _build_ui(self) -> None:
        frame = tk.Frame(self.root, padx=12, pady=12)
        frame.pack(fill="both", expand=True)

        tk.Label(
            frame,
            text="Solve XYZ using only the first 3 links, measured at the midpoint of link 3.",
            anchor="w",
            justify="left",
        ).grid(row=0, column=0, columnspan=6, sticky="w", pady=(0, 12))

        labels = [("X (mm)", "x"), ("Y (mm)", "y"), ("Z (mm)", "z")]
        for column_index, (label_text, key) in enumerate(labels):
            tk.Label(frame, text=label_text).grid(row=1, column=column_index, sticky="w")
            tk.Entry(frame, width=10, textvariable=self.target_vars[key]).grid(
                row=2, column=column_index, padx=(0, 8), sticky="w"
            )

        jog_frame = tk.Frame(frame, padx=8, pady=8, relief="groove", bd=1)
        jog_frame.grid(row=1, column=4, rowspan=4, sticky="nw", padx=(12, 0))
        tk.Label(jog_frame, text="Jog target by 10 mm").grid(row=0, column=0, columnspan=3, sticky="w", pady=(0, 6))

        tk.Button(jog_frame, text="Y+", width=6, command=lambda: self.nudge_target("y", POSITION_STEP_MM)).grid(
            row=1, column=1, padx=2, pady=2
        )
        tk.Button(jog_frame, text="X-", width=6, command=lambda: self.nudge_target("x", -POSITION_STEP_MM)).grid(
            row=2, column=0, padx=2, pady=2
        )
        tk.Label(jog_frame, text="XY", width=6, anchor="center").grid(row=2, column=1, padx=2, pady=2)
        tk.Button(jog_frame, text="X+", width=6, command=lambda: self.nudge_target("x", POSITION_STEP_MM)).grid(
            row=2, column=2, padx=2, pady=2
        )
        tk.Button(jog_frame, text="Y-", width=6, command=lambda: self.nudge_target("y", -POSITION_STEP_MM)).grid(
            row=3, column=1, padx=2, pady=2
        )

        tk.Label(jog_frame, text="Z", anchor="w").grid(row=4, column=0, columnspan=3, sticky="w", pady=(10, 2))
        tk.Button(jog_frame, text="Z+", width=6, command=lambda: self.nudge_target("z", POSITION_STEP_MM)).grid(
            row=5, column=1, padx=2, pady=2
        )
        tk.Button(jog_frame, text="Z-", width=6, command=lambda: self.nudge_target("z", -POSITION_STEP_MM)).grid(
            row=6, column=1, padx=2, pady=2
        )

        tk.Button(frame, text="Connect / Reconnect", command=self._connect_and_refresh).grid(
            row=3, column=0, columnspan=2, sticky="w", pady=(12, 0)
        )
        tk.Button(frame, text="Read Current Pose", command=self.read_current_pose).grid(
            row=3, column=2, columnspan=2, sticky="w", pady=(12, 0)
        )
        tk.Button(frame, text="Home", command=self.home_all).grid(row=3, column=3, sticky="w", pady=(12, 0))

        tk.Button(frame, text="Preview XYZ", command=self.preview_ik).grid(
            row=4, column=0, columnspan=2, sticky="w", pady=(8, 0)
        )
        tk.Button(frame, text="Move", command=self.move_to_preview).grid(
            row=4, column=2, columnspan=2, sticky="w", pady=(8, 0)
        )

        tk.Label(frame, text="Current 3-link pose", anchor="w").grid(row=5, column=0, columnspan=4, sticky="w", pady=(16, 4))
        for offset, key in enumerate(("x", "y", "z")):
            tk.Label(frame, text=f"{key.upper()}:", width=8).grid(row=6 + offset, column=0, sticky="w")
            tk.Label(frame, textvariable=self.pose_vars[key], width=24, anchor="w").grid(
                row=6 + offset, column=1, columnspan=2, sticky="w"
            )

        table_row = 6
        tk.Label(frame, text="Solved joints", anchor="w").grid(row=table_row, column=3, columnspan=2, sticky="w", pady=(0, 4))
        tk.Label(frame, text="Servo", width=8).grid(row=table_row + 1, column=3, sticky="w")
        tk.Label(frame, text="Command / Model", width=22).grid(row=table_row + 1, column=4, sticky="w")

        for servo_index in range(3):
            row_index = table_row + 2 + servo_index
            angle_var = tk.StringVar(value="--")
            self.solution_rows.append({"angle_var": angle_var})
            tk.Label(frame, text=f"Servo {servo_index + 1}", width=8).grid(row=row_index, column=3, sticky="w", pady=2)
            tk.Label(frame, textvariable=angle_var, width=22, anchor="w").grid(row=row_index, column=4, sticky="w")

        tk.Label(frame, textvariable=self.status_var, anchor="w", justify="left").grid(
            row=table_row + 6, column=0, columnspan=5, sticky="w", pady=(16, 0)
        )

    def _parse_target_entries(self) -> tuple[float, float, float]:
        try:
            return (
                float(self.target_vars["x"].get()),
                float(self.target_vars["y"].get()),
                float(self.target_vars["z"].get()),
            )
        except ValueError as exc:
            raise ValueError("Enter numeric X, Y, and Z values.") from exc

    def _connect_and_refresh(self) -> None:
        try:
            self.status_var.set("Connecting to positional bridge...")
            self.client.disconnect()
            self.client.connect()
            self.read_current_pose()
            self.status_var.set(f"Connected on {self.client.serial.port}")
        except Exception as exc:
            self.status_var.set("Connection failed")
            messagebox.showerror("Serial bridge error", str(exc))

    def nudge_target(self, axis_key: str, delta_mm: float) -> None:
        try:
            current_value = float(self.target_vars[axis_key].get())
        except ValueError:
            current_value = 0.0
        self.target_vars[axis_key].set(f"{current_value + delta_mm:.1f}")
        self.status_var.set(f"Adjusted {axis_key.upper()} target by {delta_mm:.0f} mm.")

    def _refresh_current_state(self) -> None:
        states = self.client.get_all_states()
        servo_angles: list[float] = INITIAL_ANGLES[:]
        for servo_index in range(len(servo_angles)):
            state = states.get(servo_index)
            if state is not None:
                servo_angles[servo_index] = float(state["angle"])
        self.current_servo_angles = servo_angles
        self.current_pose = forward_kinematics_from_model(servo_to_model_degrees(self.current_servo_angles))

    def _set_pose_display(self, pose: CartesianPose) -> None:
        self.pose_vars["x"].set(f"{pose.x_mm:.1f} mm")
        self.pose_vars["y"].set(f"{pose.y_mm:.1f} mm")
        self.pose_vars["z"].set(f"{pose.z_mm:.1f} mm")

    def _set_solution_display(self, solution: IKSolution | None) -> None:
        if solution is None:
            for row in self.solution_rows:
                row["angle_var"].set("--")
            return

        for servo_index, row in enumerate(self.solution_rows):
            row["angle_var"].set(
                f"{solution.servo_angles_deg[servo_index]} deg / {solution.model_angles_deg[servo_index]:.1f} deg"
            )

    def read_current_pose(self) -> None:
        try:
            self._refresh_current_state()
            if self.current_pose is None:
                raise RuntimeError("Current pose is unavailable.")
            self._set_pose_display(self.current_pose)
            self.status_var.set("Read current 3-link joint state and computed XYZ pose.")
        except Exception as exc:
            self.status_var.set("Failed to read current pose")
            messagebox.showerror("Read pose failed", str(exc))

    def preview_ik(self) -> None:
        try:
            if self.client.serial.handle is None:
                self.client.connect()
            self._refresh_current_state()
            target_x, target_y, target_z = self._parse_target_entries()
            solution, all_solutions = solve_xyz_inverse_kinematics(
                target_x,
                target_y,
                target_z,
                self.current_servo_angles,
            )
            self.preview_solution = solution
            self._set_solution_display(solution)
            self.status_var.set(
                f"Preview ready: {solution.branch_label}, "
                f"{len(all_solutions)} valid branch(es), "
                f"pos err {solution.position_error_mm:.2f} mm."
            )
        except Exception as exc:
            self.preview_solution = None
            self._set_solution_display(None)
            self.status_var.set("Preview failed")
            messagebox.showerror("XYZ preview failed", str(exc))

    def move_to_preview(self) -> None:
        if self.preview_solution is None:
            messagebox.showwarning("No preview", "Preview a target first so the move uses a validated XYZ solution.")
            return

        try:
            for servo_index in range(3):
                self.client.set_angle(servo_index, self.preview_solution.servo_angles_deg[servo_index])
            self._refresh_current_state()
            if self.current_pose is not None:
                self._set_pose_display(self.current_pose)
            self.status_var.set("Move complete. Updated the first 3 servos to the previewed XYZ target.")
        except Exception as exc:
            self.status_var.set("Move failed")
            messagebox.showerror("Move failed", str(exc))

    def home_all(self) -> None:
        try:
            self.client.home_all()
            self.preview_solution = None
            self._set_solution_display(None)
            self._refresh_current_state()
            if self.current_pose is not None:
                self._set_pose_display(self.current_pose)
            self.status_var.set("Returned all servos to the configured home position.")
        except Exception as exc:
            self.status_var.set("Home failed")
            messagebox.showerror("Home failed", str(exc))

    def on_close(self) -> None:
        self.client.disconnect()
        self.root.destroy()


def main() -> None:
    root = tk.Tk()
    CartesianIKApp(root)
    root.mainloop()


if __name__ == "__main__":
    main()
