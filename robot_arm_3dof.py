import math
import time
from collections import deque
from collections.abc import Callable, Iterable, Iterator
from dataclasses import dataclass
from itertools import cycle
from typing import Sequence

import matplotlib.pyplot as plt
import numpy as np
from matplotlib.animation import FuncAnimation
from mpl_toolkits.mplot3d import Axes3D  # noqa: F401


AnglesLike = Sequence[float]


@dataclass
class FrameTelemetry:
    joint_angles: tuple[float, float, float]
    points: list[tuple[float, float, float]] | None = None
    joint_velocity: np.ndarray | None = None
    joint_acceleration: np.ndarray | None = None
    joint_torque: np.ndarray | None = None
    end_effector_speed: float | None = None
    end_effector_acceleration: float | None = None


AngleFrame = AnglesLike | FrameTelemetry
AngleSource = Iterable[AngleFrame] | Callable[[], AngleFrame]

BASE_LENGTH_MM = 100.0
LINK_2_LENGTH_MM = 100.0
LINK_3_LENGTH_MM = 120.0

TRACE_MAX_POINTS = 300
TELEMETRY_WINDOW_SECONDS = 8.0
GRAPH_UPDATE_EVERY_N_FRAMES = 1


def _as_radians(joint_angles: AnglesLike, degrees: bool) -> tuple[float, float, float]:
    if len(joint_angles) != 3:
        raise ValueError("Expected exactly 3 joint angles: [base, shoulder, elbow].")

    q1, q2, q3 = joint_angles
    if degrees:
        q1, q2, q3 = map(math.radians, (q1, q2, q3))
    return q1, q2, q3


def forward_kinematics(
    joint_angles: AnglesLike,
    link_lengths: AnglesLike = (LINK_2_LENGTH_MM, LINK_3_LENGTH_MM),
    base_height: float = BASE_LENGTH_MM,
    degrees: bool = True,
) -> list[tuple[float, float, float]]:
    if len(link_lengths) != 2:
        raise ValueError("Expected exactly 2 arm link lengths: [link_2, link_3].")

    l1, l2 = link_lengths
    q1, q2, q3 = _as_radians(joint_angles, degrees)

    radial_1 = l1 * math.cos(q2)
    z_1 = base_height + l1 * math.sin(q2)

    radial_2 = radial_1 + l2 * math.cos(q2 + q3)
    z_2 = z_1 + l2 * math.sin(q2 + q3)

    x_1 = radial_1 * math.cos(q1)
    y_1 = radial_1 * math.sin(q1)

    x_2 = radial_2 * math.cos(q1)
    y_2 = radial_2 * math.sin(q1)

    base_center = (0.0, 0.0, 0.0)
    shoulder = (0.0, 0.0, base_height)
    elbow = (x_1, y_1, z_1)
    end_effector = (x_2, y_2, z_2)
    return [base_center, shoulder, elbow, end_effector]


def _set_axes_equal(ax: plt.Axes, radius: float, z_max: float) -> None:
    ax.set_xlim(-radius, radius)
    ax.set_ylim(-radius, radius)
    ax.set_zlim(-20.0, z_max)
    ax.set_box_aspect((1.0, 1.0, max(0.6, z_max / (2.0 * radius))))


def _configure_robot_axes(ax: plt.Axes, link_lengths: AnglesLike, base_height: float) -> None:
    reach = base_height + sum(link_lengths)
    radius = reach + 40.0
    z_max = reach + 40.0
    ax.set_title("3DOF Robot Arm")
    ax.set_xlabel("X (mm)")
    ax.set_ylabel("Y (mm)")
    ax.set_zlabel("Z (mm)")
    _set_axes_equal(ax, radius, z_max)


def _update_line3d(line: object, start: Sequence[float], end: Sequence[float]) -> None:
    line.set_data_3d([start[0], end[0]], [start[1], end[1]], [start[2], end[2]])


def _update_polyline3d(line: object, points: Sequence[Sequence[float]]) -> None:
    if not points:
        line.set_data_3d([], [], [])
        return
    line.set_data_3d(
        [point[0] for point in points],
        [point[1] for point in points],
        [point[2] for point in points],
    )


def _update_points3d(scatter: object, points: Sequence[Sequence[float]]) -> None:
    scatter._offsets3d = (
        [point[0] for point in points],
        [point[1] for point in points],
        [point[2] for point in points],
    )


def _coerce_frame_data(
    frame: AngleFrame,
    link_lengths: AnglesLike,
    base_height: float,
    degrees: bool,
) -> FrameTelemetry:
    if isinstance(frame, FrameTelemetry):
        if frame.points is None:
            frame.points = forward_kinematics(frame.joint_angles, link_lengths, base_height, degrees)
        return frame
    return FrameTelemetry(
        joint_angles=tuple(frame),
        points=forward_kinematics(frame, link_lengths, base_height, degrees),
    )


def plot_robot_arm(
    joint_angles: AnglesLike,
    link_lengths: AnglesLike = (LINK_2_LENGTH_MM, LINK_3_LENGTH_MM),
    base_height: float = BASE_LENGTH_MM,
    degrees: bool = True,
    show_end_effector_trace: bool = False,
) -> None:
    fig = plt.figure(figsize=(7, 6))
    ax = fig.add_subplot(111, projection="3d")
    _configure_robot_axes(ax, link_lengths, base_height)

    base_center, shoulder, elbow, end_effector = forward_kinematics(
        joint_angles, link_lengths, base_height, degrees
    )
    ax.plot(
        [base_center[0], shoulder[0]],
        [base_center[1], shoulder[1]],
        [base_center[2], shoulder[2]],
        color="red",
        linewidth=4,
    )
    ax.plot(
        [shoulder[0], elbow[0]],
        [shoulder[1], elbow[1]],
        [shoulder[2], elbow[2]],
        color="green",
        linewidth=4,
    )
    ax.plot(
        [elbow[0], end_effector[0]],
        [elbow[1], end_effector[1]],
        [elbow[2], end_effector[2]],
        color="blue",
        linewidth=4,
    )
    ax.scatter(
        [shoulder[0], elbow[0]],
        [shoulder[1], elbow[1]],
        [shoulder[2], elbow[2]],
        s=10,
        color="black",
    )
    ax.scatter([end_effector[0]], [end_effector[1]], [end_effector[2]], s=14, color="orange")
    if show_end_effector_trace:
        _update_polyline3d(ax.plot([], [], [], linestyle="--", color="tab:purple")[0], [end_effector])

    plt.tight_layout()
    plt.show()


def animate_robot_arm(
    angle_source: AngleSource,
    interval_ms: int = 100,
    link_lengths: AnglesLike = (LINK_2_LENGTH_MM, LINK_3_LENGTH_MM),
    base_height: float = BASE_LENGTH_MM,
    degrees: bool = True,
    trace_end_effector: bool = True,
) -> FuncAnimation:
    fig = plt.figure(figsize=(13, 10))
    grid = fig.add_gridspec(4, 2, height_ratios=[2.6, 1.0, 1.0, 1.0])
    ax = fig.add_subplot(grid[0, :], projection="3d")
    ee_ax = fig.add_subplot(grid[1, 0])
    joint_vel_ax = fig.add_subplot(grid[1, 1], sharex=ee_ax)
    joint_compare_axes = [
        fig.add_subplot(grid[2, 0], sharex=ee_ax),
        fig.add_subplot(grid[2, 1], sharex=ee_ax),
        fig.add_subplot(grid[3, :], sharex=ee_ax),
    ]
    joint_torque_axes = [compare_ax.twinx() for compare_ax in joint_compare_axes]

    iterator: Iterator[AnglesLike] | None = None
    callback: Callable[[], AnglesLike] | None = None

    if callable(angle_source):
        callback = angle_source
        initial_frame = callback()
    else:
        iterator = cycle(angle_source)
        initial_frame = next(iterator)

    initial_data = _coerce_frame_data(initial_frame, link_lengths, base_height, degrees)
    initial_angles = initial_data.joint_angles
    initial_points = initial_data.points
    initial_angles_rad = np.array(_as_radians(initial_angles, degrees), dtype=float)
    initial_ee = np.array(initial_points[-1], dtype=float)

    trace_points: deque[tuple[float, float, float]] = deque(maxlen=TRACE_MAX_POINTS)
    if trace_end_effector:
        trace_points.append(initial_points[-1])

    time_history: deque[float] = deque([0.0], maxlen=TRACE_MAX_POINTS)
    ee_speed_history: deque[float] = deque([0.0], maxlen=TRACE_MAX_POINTS)
    ee_acc_history: deque[float] = deque([0.0], maxlen=TRACE_MAX_POINTS)
    joint_velocity_history: deque[np.ndarray] = deque([np.zeros(3, dtype=float)], maxlen=TRACE_MAX_POINTS)
    joint_acc_history: deque[np.ndarray] = deque([np.zeros(3, dtype=float)], maxlen=TRACE_MAX_POINTS)
    joint_torque_history: deque[np.ndarray] = deque([np.zeros(3, dtype=float)], maxlen=TRACE_MAX_POINTS)

    state = {
        "last_time": time.perf_counter(),
        "last_angles_rad": initial_angles_rad,
        "last_joint_velocity": (
            np.radians(initial_data.joint_velocity)
            if degrees and initial_data.joint_velocity is not None
            else (
                initial_data.joint_velocity.copy()
                if initial_data.joint_velocity is not None
                else np.zeros(3, dtype=float)
            )
        ),
        "last_ee": initial_ee,
        "frame_count": 0,
    }

    _configure_robot_axes(ax, link_lengths, base_height)
    base_line, = ax.plot([], [], [], color="red", linewidth=4)
    link2_line, = ax.plot([], [], [], color="green", linewidth=4)
    link3_line, = ax.plot([], [], [], color="blue", linewidth=4)
    trace_line, = ax.plot([], [], [], linewidth=1.5, linestyle="--", color="tab:purple")
    joint_scatter = ax.scatter([], [], [], s=10, color="black")
    ee_scatter = ax.scatter([], [], [], s=14, color="orange")

    _update_line3d(base_line, initial_points[0], initial_points[1])
    _update_line3d(link2_line, initial_points[1], initial_points[2])
    _update_line3d(link3_line, initial_points[2], initial_points[3])
    _update_points3d(joint_scatter, [initial_points[1], initial_points[2]])
    _update_points3d(ee_scatter, [initial_points[3]])
    _update_polyline3d(trace_line, list(trace_points))

    ee_vel_line, = ee_ax.plot(list(time_history), list(ee_speed_history), color="tab:blue", label="|v_ee|")
    ee_acc_line, = ee_ax.plot(list(time_history), list(ee_acc_history), color="tab:red", label="|a_ee|")
    ee_ax.set_ylabel("mm/s, mm/s^2")
    ee_ax.set_title("End Effector Magnitude")
    ee_ax.grid(True, alpha=0.3)
    ee_ax.legend(loc="upper right")

    joint_colors = ("red", "green", "blue")
    joint_labels = ("q1", "q2", "q3")
    joint_vel_lines = []
    joint_compare_acc_lines = []
    joint_compare_torque_lines = []
    for index, (color, label) in enumerate(zip(joint_colors, joint_labels)):
        vel_line, = joint_vel_ax.plot(
            list(time_history),
            [joint_velocity_history[0][index]],
            color=color,
            label=f"{label}_dot",
        )
        acc_line, = joint_compare_axes[index].plot(
            list(time_history),
            [joint_acc_history[0][index]],
            color=color,
            label=f"{label}_ddot",
            linewidth=1.8,
        )
        joint_vel_lines.append(vel_line)
        joint_compare_acc_lines.append(acc_line)
        torque_line, = joint_torque_axes[index].plot(
            list(time_history),
            [joint_torque_history[0][index]],
            color="black",
            linestyle="--",
            label=f"{label}_tau",
            linewidth=1.6,
        )
        joint_compare_torque_lines.append(torque_line)

    joint_vel_ax.set_ylabel("deg/s" if degrees else "rad/s")
    joint_vel_ax.set_title("Joint Angular Velocity")
    joint_vel_ax.grid(True, alpha=0.3)
    joint_vel_ax.legend(loc="upper right")
    for index, compare_ax in enumerate(joint_compare_axes):
        compare_ax.set_title(f"{joint_labels[index]} Acceleration vs Torque")
        compare_ax.set_ylabel("deg/s^2" if degrees else "rad/s^2")
        compare_ax.set_xlabel("Time (s)")
        compare_ax.grid(True, alpha=0.3)
        joint_torque_axes[index].set_ylabel("N*m")
        compare_ax.legend(loc="upper left")
        joint_torque_axes[index].legend(loc="upper right")

    telemetry_axes = (ee_ax, joint_vel_ax, *joint_compare_axes, *joint_torque_axes)

    def _refresh_telemetry_axes() -> None:
        current_time = time_history[-1]
        x_min = max(0.0, current_time - TELEMETRY_WINDOW_SECONDS)
        x_max = max(5.0, current_time + 0.1)
        for telemetry_ax in telemetry_axes:
            telemetry_ax.set_xlim(x_min, x_max)
            telemetry_ax.relim()
            telemetry_ax.autoscale_view(scaley=True)

    def update(_frame: int) -> None:
        nonlocal iterator
        frame = callback() if callback is not None else next(iterator)  # type: ignore[arg-type]
        frame_data = _coerce_frame_data(frame, link_lengths, base_height, degrees)
        angles = frame_data.joint_angles

        now = time.perf_counter()
        dt = max(now - state["last_time"], interval_ms / 1000.0)
        state["last_time"] = now

        points = frame_data.points
        end_effector = np.array(points[-1], dtype=float)
        angles_rad = np.array(_as_radians(angles, degrees), dtype=float)

        if frame_data.joint_velocity is not None:
            if degrees:
                joint_velocity = np.radians(frame_data.joint_velocity)
                joint_velocity_plot = frame_data.joint_velocity
            else:
                joint_velocity = frame_data.joint_velocity
                joint_velocity_plot = frame_data.joint_velocity
        else:
            joint_velocity = (angles_rad - state["last_angles_rad"]) / dt
            joint_velocity_plot = np.degrees(joint_velocity) if degrees else joint_velocity

        if frame_data.joint_acceleration is not None:
            if degrees:
                joint_acceleration_plot = frame_data.joint_acceleration
            else:
                joint_acceleration_plot = frame_data.joint_acceleration
        else:
            joint_acceleration = (joint_velocity - state["last_joint_velocity"]) / dt
            joint_acceleration_plot = (
                np.degrees(joint_acceleration) if degrees else joint_acceleration
            )

        if frame_data.end_effector_speed is not None:
            ee_speed = frame_data.end_effector_speed
        else:
            ee_speed = float(np.linalg.norm((end_effector - state["last_ee"]) / dt))

        if frame_data.end_effector_acceleration is not None:
            ee_acceleration = frame_data.end_effector_acceleration
        else:
            ee_acceleration = abs((ee_speed - ee_speed_history[-1]) / dt)

        joint_torque_plot = (
            frame_data.joint_torque if frame_data.joint_torque is not None else np.zeros(3, dtype=float)
        )

        time_history.append(time_history[-1] + dt)
        ee_speed_history.append(ee_speed)
        ee_acc_history.append(ee_acceleration)
        joint_velocity_history.append(joint_velocity_plot)
        joint_acc_history.append(joint_acceleration_plot)
        joint_torque_history.append(joint_torque_plot)

        state["last_angles_rad"] = angles_rad
        state["last_joint_velocity"] = joint_velocity
        state["last_ee"] = end_effector

        _update_line3d(base_line, points[0], points[1])
        _update_line3d(link2_line, points[1], points[2])
        _update_line3d(link3_line, points[2], points[3])
        _update_points3d(joint_scatter, [points[1], points[2]])
        _update_points3d(ee_scatter, [points[3]])

        if trace_end_effector:
            trace_points.append(points[3])
            _update_polyline3d(trace_line, list(trace_points))

        state["frame_count"] += 1
        if state["frame_count"] % GRAPH_UPDATE_EVERY_N_FRAMES == 0:
            history_times = list(time_history)
            ee_vel_line.set_data(history_times, list(ee_speed_history))
            ee_acc_line.set_data(history_times, list(ee_acc_history))

            joint_velocity_series = np.array(joint_velocity_history)
            joint_acc_series = np.array(joint_acc_history)
            joint_torque_series = np.array(joint_torque_history)
            for index, vel_line in enumerate(joint_vel_lines):
                vel_line.set_data(history_times, joint_velocity_series[:, index])
            for index, acc_line in enumerate(joint_compare_acc_lines):
                acc_line.set_data(history_times, joint_acc_series[:, index])
            for index, torque_line in enumerate(joint_compare_torque_lines):
                torque_line.set_data(history_times, joint_torque_series[:, index])
            _refresh_telemetry_axes()

    animation = FuncAnimation(fig, update, interval=interval_ms, cache_frame_data=False)
    plt.tight_layout()
    plt.show()
    return animation


if __name__ == "__main__":
    demo_angles = [
        (0, 15, 10),
        (20, 25, 0),
        (40, 35, -10),
        (60, 45, -20),
        (80, 30, 15),
        (100, 20, 20),
    ]
    animate_robot_arm(demo_angles, interval_ms=500, degrees=True, trace_end_effector=True)
