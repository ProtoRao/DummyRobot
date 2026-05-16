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