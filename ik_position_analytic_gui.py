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
SERVO_MODEL_OFFSETS = [0.0, 0.0, 0.0, 0.0, 0.0]
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
    servo_angles_deg: list[float]


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

def solve_xyz_inverse_kinematics(
    x: float,
    y: float,
    z: float,
) -> IKSolution:
    
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

    return IKSolution(model_angles_deg=[math.degrees(theta1), math.degrees(theta2), math.degrees(theta3), 0, math.degrees(theta4)], servo_angles_deg=[math.degrees(theta1), math.degrees(theta2), math.degrees(theta3), 0, math.degrees(theta4)])

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
            "x": tk.StringVar(value="-7.95"),
            "y": tk.StringVar(value="196.06"),
            "z": tk.StringVar(value="228.075"),
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

#     def _build_ui(self) -> None:
#         frame = tk.Frame(self.root, padx=12, pady=12)
#         frame.pack(fill="both", expand=True)

#         tk.Label(
#             frame,
#             text="Solve XYZ using only the first 3 links, measured at the midpoint of link 3.",
#             anchor="w",
#             justify="left",
#         ).grid(row=0, column=0, columnspan=6, sticky="w", pady=(0, 12))

#         labels = [("X (mm)", "x"), ("Y (mm)", "y"), ("Z (mm)", "z")]
#         for column_index, (label_text, key) in enumerate(labels):
#             tk.Label(frame, text=label_text).grid(row=1, column=column_index, sticky="w")
#             tk.Entry(frame, width=10, textvariable=self.target_vars[key]).grid(
#                 row=2, column=column_index, padx=(0, 8), sticky="w"
#             )

#         jog_frame = tk.Frame(frame, padx=8, pady=8, relief="groove", bd=1)
#         jog_frame.grid(row=1, column=4, rowspan=4, sticky="nw", padx=(12, 0))
#         tk.Label(jog_frame, text="Jog target by 10 mm").grid(row=0, column=0, columnspan=3, sticky="w", pady=(0, 6))

#         tk.Button(jog_frame, text="Y+", width=6, command=lambda: self.nudge_target("y", POSITION_STEP_MM)).grid(
#             row=1, column=1, padx=2, pady=2
#         )
#         tk.Button(jog_frame, text="X-", width=6, command=lambda: self.nudge_target("x", -POSITION_STEP_MM)).grid(
#             row=2, column=0, padx=2, pady=2
#         )
#         tk.Label(jog_frame, text="XY", width=6, anchor="center").grid(row=2, column=1, padx=2, pady=2)
#         tk.Button(jog_frame, text="X+", width=6, command=lambda: self.nudge_target("x", POSITION_STEP_MM)).grid(
#             row=2, column=2, padx=2, pady=2
#         )
#         tk.Button(jog_frame, text="Y-", width=6, command=lambda: self.nudge_target("y", -POSITION_STEP_MM)).grid(
#             row=3, column=1, padx=2, pady=2
#         )

#         tk.Label(jog_frame, text="Z", anchor="w").grid(row=4, column=0, columnspan=3, sticky="w", pady=(10, 2))
#         tk.Button(jog_frame, text="Z+", width=6, command=lambda: self.nudge_target("z", POSITION_STEP_MM)).grid(
#             row=5, column=1, padx=2, pady=2
#         )
#         tk.Button(jog_frame, text="Z-", width=6, command=lambda: self.nudge_target("z", -POSITION_STEP_MM)).grid(
#             row=6, column=1, padx=2, pady=2
#         )

#         tk.Button(frame, text="Connect / Reconnect", command=self._connect_and_refresh).grid(
#             row=3, column=0, columnspan=2, sticky="w", pady=(12, 0)
#         )
#         tk.Button(frame, text="Read Current Pose", command=self.read_current_pose).grid(
#             row=3, column=2, columnspan=2, sticky="w", pady=(12, 0)
#         )
#         tk.Button(frame, text="Home", command=self.home_all).grid(row=3, column=3, sticky="w", pady=(12, 0))

#         tk.Button(frame, text="Preview XYZ", command=self.preview_ik).grid(
#             row=4, column=0, columnspan=2, sticky="w", pady=(8, 0)
#         )
#         tk.Button(frame, text="Move", command=self.move_to_preview).grid(
#             row=4, column=2, columnspan=2, sticky="w", pady=(8, 0)
#         )

#         tk.Label(frame, text="Current 3-link pose", anchor="w").grid(row=5, column=0, columnspan=4, sticky="w", pady=(16, 4))
#         for offset, key in enumerate(("x", "y", "z")):
#             tk.Label(frame, text=f"{key.upper()}:", width=8).grid(row=6 + offset, column=0, sticky="w")
#             tk.Label(frame, textvariable=self.pose_vars[key], width=24, anchor="w").grid(
#                 row=6 + offset, column=1, columnspan=2, sticky="w"
#             )

#         table_row = 6
#         tk.Label(frame, text="Solved joints", anchor="w").grid(row=table_row, column=3, columnspan=2, sticky="w", pady=(0, 4))
#         tk.Label(frame, text="Servo", width=8).grid(row=table_row + 1, column=3, sticky="w")
#         tk.Label(frame, text="Command / Model", width=22).grid(row=table_row + 1, column=4, sticky="w")

#         for servo_index in range(5):
#             row_index = table_row + 2 + servo_index
#             angle_var = tk.StringVar(value="--")
#             self.solution_rows.append({"angle_var": angle_var})
#             tk.Label(frame, text=f"Servo {servo_index + 1}", width=8).grid(row=row_index, column=3, sticky="w", pady=2)
#             tk.Label(frame, textvariable=angle_var, width=22, anchor="w").grid(row=row_index, column=4, sticky="w")

#         tk.Label(frame, textvariable=self.status_var, anchor="w", justify="left").grid(
#             row=table_row + 6, column=0, columnspan=5, sticky="w", pady=(16, 0)
#         )

#     def _parse_target_entries(self) -> tuple[float, float, float]:
#         try:
#             return (
#                 float(self.target_vars["x"].get()),
#                 float(self.target_vars["y"].get()),
#                 float(self.target_vars["z"].get()),
#             )
#         except ValueError as exc:
#             raise ValueError("Enter numeric X, Y, and Z values.") from exc

#     def _connect_and_refresh(self) -> None:
#         try:
#             self.status_var.set("Connecting to positional bridge...")
#             self.client.disconnect()
#             self.client.connect()
#             self.read_current_pose()
#             self.status_var.set(f"Connected on {self.client.serial.port}")
#         except Exception as exc:
#             self.status_var.set("Connection failed")
#             messagebox.showerror("Serial bridge error", str(exc))

#     def nudge_target(self, axis_key: str, delta_mm: float) -> None:
#         try:
#             current_value = float(self.target_vars[axis_key].get())
#         except ValueError:
#             current_value = 0.0
#         self.target_vars[axis_key].set(f"{current_value + delta_mm:.1f}")
#         self.status_var.set(f"Adjusted {axis_key.upper()} target by {delta_mm:.0f} mm.")

#     def _refresh_current_state(self) -> None:
#         states = self.client.get_all_states()
#         servo_angles: list[float] = INITIAL_ANGLES[:]
#         for servo_index in range(len(servo_angles)):
#             state = states.get(servo_index)
#             if state is not None:
#                 servo_angles[servo_index] = float(state["angle"])
#         self.current_servo_angles = servo_angles
#         self.current_pose = forward_kinematics_from_model(servo_to_model_degrees(self.current_servo_angles))

#     def _set_pose_display(self, pose: CartesianPose) -> None:
#         self.pose_vars["x"].set(f"{pose.x_mm:.1f} mm")
#         self.pose_vars["y"].set(f"{pose.y_mm:.1f} mm")
#         self.pose_vars["z"].set(f"{pose.z_mm:.1f} mm")

#     def _set_solution_display(self, solution: IKSolution | None) -> None:
#         if solution is None:
#             for row in self.solution_rows:
#                 row["angle_var"].set("--")
#             return

#         for servo_index, row in enumerate(self.solution_rows):
#             row["angle_var"].set(
#                 f"{solution.servo_angles_deg[servo_index]} deg / {solution.model_angles_deg[servo_index]:.1f} deg"
#             )

#     def read_current_pose(self) -> None:
#         try:
#             self._refresh_current_state()
#             if self.current_pose is None:
#                 raise RuntimeError("Current pose is unavailable.")
#             self._set_pose_display(self.current_pose)
#             self.status_var.set("Read current 3-link joint state and computed XYZ pose.")
#         except Exception as exc:
#             self.status_var.set("Failed to read current pose")
#             messagebox.showerror("Read pose failed", str(exc))

#     def preview_ik(self) -> None:
#         try:
#             if self.client.serial.handle is None:
#                 self.client.connect()
#             self._refresh_current_state()
#             target_x, target_y, target_z = self._parse_target_entries()
#             solution = solve_xyz_inverse_kinematics(
#                 target_x,
#                 target_y,
#                 target_z
#             )
#             self.preview_solution = solution
#             self._set_solution_display(solution)
#             self.status_var.set(
#                 f"Preview ready"
#             )
#         except Exception as exc:
#             self.preview_solution = None
#             self._set_solution_display(None)
#             self.status_var.set("Preview failed")
#             messagebox.showerror("XYZ preview failed", str(exc))

#     def move_to_preview(self) -> None:
#         if self.preview_solution is None:
#             messagebox.showwarning("No preview", "Preview a target first so the move uses a validated XYZ solution.")
#             return

#         try:
#             for servo_index in range(5):
#                 self.client.set_angle(servo_index, self.preview_solution.servo_angles_deg[servo_index])
#             self._refresh_current_state()
#             if self.current_pose is not None:
#                 self._set_pose_display(self.current_pose)
#             self.status_var.set("Move complete. Updated the first 3 servos to the previewed XYZ target.")
#         except Exception as exc:
#             self.status_var.set("Move failed")
#             messagebox.showerror("Move failed", str(exc))

#     def home_all(self) -> None:
#         try:
#             self.client.home_all()
#             self.preview_solution = None
#             self._set_solution_display(None)
#             self._refresh_current_state()
#             if self.current_pose is not None:
#                 self._set_pose_display(self.current_pose)
#             self.status_var.set("Returned all servos to the configured home position.")
#         except Exception as exc:
#             self.status_var.set("Home failed")
#             messagebox.showerror("Home failed", str(exc))

#     def on_close(self) -> None:
#         self.client.disconnect()
#         self.root.destroy()


# def main() -> None:
#     root = tk.Tk()
#     CartesianIKApp(root)
#     root.mainloop()

class CartesianIKApp:
    def __init__(self, root: tk.Tk) -> None:
        self.root = root
        self.root.title("3-Link Cartesian XYZ Controller")
        self.client = PositionalServoBridgeClient(DEFAULT_PORT)
        self.preview_solution: IKSolution | None = None
        self.current_servo_angles = INITIAL_ANGLES[:]
        
        # Preset Storage
        self.presets = [[-7.9, 196.1, 228.1] for _ in range(4)]
        self.selected_preset_idx = tk.IntVar(value=0)

        self.status_var = tk.StringVar(value="Disconnected")
        self.target_vars = {
            "x": tk.StringVar(value="-7.9"),
            "y": tk.StringVar(value="196.1"),
            "z": tk.StringVar(value="228.1"),
        }
        self.solution_rows: list[dict[str, tk.StringVar]] = []

        self._build_ui()
        self._connect_and_refresh()
        self.root.protocol("WM_DELETE_WINDOW", self.on_close)

    def _build_ui(self) -> None:
        frame = tk.Frame(self.root, padx=12, pady=12)
        frame.pack(fill="both", expand=True)

        tk.Label(frame, text="IK Controller with Programmed Presets", font=("Arial", 10, "bold")).grid(row=0, column=0, columnspan=3, sticky="w", pady=(0, 10))

        # Inputs
        for i, (label, key) in enumerate([("X (mm)", "x"), ("Y (mm)", "y"), ("Z (mm)", "z")]):
            tk.Label(frame, text=label).grid(row=1, column=i, sticky="w")
            tk.Entry(frame, width=10, textvariable=self.target_vars[key]).grid(row=2, column=i, padx=(0, 8), sticky="w")

        # Jogging
        jog_frame = tk.Frame(frame, padx=8, pady=8, relief="groove", bd=1)
        jog_frame.grid(row=1, column=4, rowspan=5, sticky="nw", padx=(12, 0))
        tk.Label(jog_frame, text="Jog (10mm)").grid(row=0, column=1)
        tk.Button(jog_frame, text="Y+", command=lambda: self.nudge_target("y", 10)).grid(row=1, column=1)
        tk.Button(jog_frame, text="X-", command=lambda: self.nudge_target("x", -10)).grid(row=2, column=0)
        tk.Button(jog_frame, text="X+", command=lambda: self.nudge_target("x", 10)).grid(row=2, column=2)
        tk.Button(jog_frame, text="Y-", command=lambda: self.nudge_target("y", -10)).grid(row=3, column=1)
        tk.Button(jog_frame, text="Z+", command=lambda: self.nudge_target("z", 10)).grid(row=4, column=1, pady=(5,0))
        tk.Button(jog_frame, text="Z-", command=lambda: self.nudge_target("z", -10)).grid(row=5, column=1)

        # Connection / Global Controls
        tk.Button(frame, text="Reconnect", command=self._connect_and_refresh).grid(row=3, column=0, sticky="w", pady=5)
        tk.Button(frame, text="Home All", command=self.home_all).grid(row=3, column=1, sticky="w", pady=5)
        tk.Button(frame, text="Preview", command=self.preview_ik).grid(row=4, column=0, sticky="w")
        tk.Button(frame, text="Move", command=self.move_to_preview).grid(row=4, column=1, sticky="w")

        # Presets Section
        p_frame = tk.LabelFrame(frame, text="Presets", padx=10, pady=10)
        p_frame.grid(row=5, column=0, columnspan=3, sticky="nsew", pady=15)
        
        for i in range(4):
            tk.Button(p_frame, text=f"Go P{i+1}", width=7, command=lambda idx=i: self.run_preset(idx)).grid(row=0, column=i, padx=2)
            tk.Radiobutton(p_frame, text=f"S{i+1}", variable=self.selected_preset_idx, value=i).grid(row=1, column=i)
        
        tk.Button(p_frame, text="Set Current to Selected Slot", command=self.save_preset).grid(row=2, column=0, columnspan=4, pady=(5,0))

        # Joint Solution Table
        tk.Label(frame, text="Joint Solutions (Deg)").grid(row=6, column=3, columnspan=2, sticky="w")
        for i in range(5):
            var = tk.StringVar(value="--")
            self.solution_rows.append({"angle_var": var})
            tk.Label(frame, text=f"S{i+1}:").grid(row=7+i, column=3, sticky="w")
            tk.Label(frame, textvariable=var).grid(row=7+i, column=4, sticky="w")

        tk.Label(frame, textvariable=self.status_var, fg="blue").grid(row=12, column=0, columnspan=5, sticky="w", pady=(10,0))

    def _parse_target_entries(self) -> tuple[float, float, float]:
        return (float(self.target_vars["x"].get()), float(self.target_vars["y"].get()), float(self.target_vars["z"].get()))

    def _connect_and_refresh(self) -> None:
        try:
            self.client.disconnect(); self.client.connect()
            self.status_var.set("Connected")
        except Exception as e:
            self.status_var.set("Conn Failed"); messagebox.showerror("Error", str(e))

    def nudge_target(self, axis: str, delta: float) -> None:
        val = float(self.target_vars[axis].get() or 0)
        self.target_vars[axis].set(f"{val + delta:.1f}")

    def preview_ik(self) -> bool:
        try:
            x, y, z = self._parse_target_entries()
            sol = solve_xyz_inverse_kinematics(x, y, z)
            self.preview_solution = sol
            if sol:
                for i, row in enumerate(self.solution_rows):
                    row["angle_var"].set(f"{sol.servo_angles_deg[i]:.1f}")
                self.status_var.set("IK Solution Found")
                return True
            else:
                for row in self.solution_rows: row["angle_var"].set("OUT")
                self.status_var.set("Target Unreachable")
                return False
        except Exception as e:
            self.status_var.set("Error"); return False

    def move_to_preview(self) -> None:
        if not self.preview_solution: return
        try:
            for i in range(5): self.client.set_angle(i, self.preview_solution.servo_angles_deg[i])
            self.status_var.set("Move Complete")
        except Exception as e: messagebox.showerror("Move Failed", str(e))

    def run_preset(self, idx: int) -> None:
        vals = self.presets[idx]
        self.target_vars["x"].set(str(vals[0]))
        self.target_vars["y"].set(str(vals[1]))
        self.target_vars["z"].set(str(vals[2]))
        if self.preview_ik(): self.move_to_preview()

    def save_preset(self) -> None:
        try:
            idx = self.selected_preset_idx.get()
            self.presets[idx] = list(self._parse_target_entries())
            self.status_var.set(f"Saved to P{idx+1}")
        except: messagebox.showerror("Error", "Invalid coordinates")

    def home_all(self) -> None:
        self.client.home_all(); self.status_var.set("Homed")

    def on_close(self) -> None:
        self.client.disconnect(); self.root.destroy()

if __name__ == "__main__":
    root = tk.Tk()
    CartesianIKApp(root)
    root.mainloop()