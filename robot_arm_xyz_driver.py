import math
import time
from dataclasses import dataclass

import numpy as np

from robot_arm_3dof import (
    BASE_LENGTH_MM,
    FrameTelemetry,
    LINK_2_LENGTH_MM,
    LINK_3_LENGTH_MM,
    animate_robot_arm,
    forward_kinematics,
)


LINK_LENGTHS = (LINK_2_LENGTH_MM, LINK_3_LENGTH_MM)
LINK_2_MASS_KG = 1.8
LINK_3_MASS_KG = 1.4
PAYLOAD_MASS_KG = 0.6
LINK_RADIUS_M = 0.005
GRAVITY_M_S2 = 9.81


@dataclass(frozen=True)
class MotionSegment:
    start: tuple[float, float, float]
    end: tuple[float, float, float]
    duration: float
    hold: bool = False


def _smoothstep_profile(tau: float) -> tuple[float, float]:
    """
    Quintic profile returning normalized position and velocity.

    Position: 0 -> 1 with zero velocity/acceleration at both ends.
    Velocity is normalized with respect to unit-duration time.
    """
    tau = max(0.0, min(1.0, tau))
    s = 10.0 * tau**3 - 15.0 * tau**4 + 6.0 * tau**5
    ds_dtau = 30.0 * tau**2 - 60.0 * tau**3 + 30.0 * tau**4
    return s, ds_dtau


class PickAndPlaceTrajectory:
    def __init__(self) -> None:
        pickup_xy = (150.0, -70.0)
        place_xy = (70.0, 150.0)
        travel_z = 190.0
        pickup_z = 45.0
        place_z = 55.0

        home = (190.0, 0.0, 170.0)
        above_pick = (pickup_xy[0], pickup_xy[1], travel_z)
        pick = (pickup_xy[0], pickup_xy[1], pickup_z)
        above_place = (place_xy[0], place_xy[1], travel_z)
        place = (place_xy[0], place_xy[1], place_z)

        self.segments = [
            MotionSegment(home, above_pick, 2.4),
            MotionSegment(above_pick, pick, 1.2),
            MotionSegment(pick, pick, 0.5, hold=True),
            MotionSegment(pick, above_pick, 1.2),
            MotionSegment(above_pick, above_place, 3.0),
            MotionSegment(above_place, place, 1.2),
            MotionSegment(place, place, 0.5, hold=True),
            MotionSegment(place, above_place, 1.2),
            MotionSegment(above_place, home, 2.6),
            MotionSegment(home, home, 0.6, hold=True),
        ]
        self.total_duration = sum(segment.duration for segment in self.segments)

    def sample(self, t_seconds: float) -> tuple[np.ndarray, np.ndarray]:
        cycle_time = t_seconds % self.total_duration
        elapsed = 0.0

        for segment in self.segments:
            if cycle_time <= elapsed + segment.duration:
                local_time = cycle_time - elapsed
                start = np.array(segment.start, dtype=float)
                end = np.array(segment.end, dtype=float)

                if segment.hold or segment.duration <= 0.0:
                    return start.copy(), np.zeros(3, dtype=float)

                tau = local_time / segment.duration
                s, ds_dtau = _smoothstep_profile(tau)
                delta = end - start
                position = start + s * delta
                velocity = (ds_dtau / segment.duration) * delta
                return position, velocity
            elapsed += segment.duration

        last = np.array(self.segments[-1].end, dtype=float)
        return last, np.zeros(3, dtype=float)


def inverse_kinematics_3dof(
    x: float,
    y: float,
    z: float,
    link_lengths: tuple[float, float] = LINK_LENGTHS,
    base_height: float = BASE_LENGTH_MM,
    elbow_up: bool = False,
) -> np.ndarray:
    l1, l2 = link_lengths
    q1 = math.atan2(y, x)

    radial = math.hypot(x, y)
    vertical = z - base_height
    distance_sq = radial * radial + vertical * vertical

    cos_q3 = (distance_sq - l1 * l1 - l2 * l2) / (2.0 * l1 * l2)
    cos_q3 = max(-1.0, min(1.0, cos_q3))

    sin_sign = 1.0 if elbow_up else -1.0
    sin_q3 = sin_sign * math.sqrt(max(0.0, 1.0 - cos_q3 * cos_q3))
    q3 = math.atan2(sin_q3, cos_q3)

    k1 = l1 + l2 * cos_q3
    k2 = l2 * sin_q3
    q2 = math.atan2(vertical, radial) - math.atan2(k2, k1)
    return np.array([q1, q2, q3], dtype=float)


def jacobian_3dof(
    joint_angles_rad: np.ndarray,
    link_lengths: tuple[float, float] = LINK_LENGTHS,
) -> np.ndarray:
    q1, q2, q3 = joint_angles_rad
    l1, l2 = link_lengths

    c1 = math.cos(q1)
    s1 = math.sin(q1)
    c2 = math.cos(q2)
    s2 = math.sin(q2)
    c23 = math.cos(q2 + q3)
    s23 = math.sin(q2 + q3)

    radial = l1 * c2 + l2 * c23
    radial_derivative_q2 = -l1 * s2 - l2 * s23
    radial_derivative_q3 = -l2 * s23

    return np.array(
        [
            [-s1 * radial, c1 * radial_derivative_q2, c1 * radial_derivative_q3],
            [c1 * radial, s1 * radial_derivative_q2, s1 * radial_derivative_q3],
            [0.0, l1 * c2 + l2 * c23, l2 * c23],
        ],
        dtype=float,
    )


def damped_least_squares_inverse(jacobian: np.ndarray, damping: float = 0.08) -> np.ndarray:
    identity = np.eye(jacobian.shape[0], dtype=float)
    return jacobian.T @ np.linalg.inv(jacobian @ jacobian.T + (damping**2) * identity)


def radians_to_degrees(joint_angles_rad: np.ndarray) -> tuple[float, float, float]:
    return tuple(float(math.degrees(angle)) for angle in joint_angles_rad)


def _pitch_axis_world(q1: float) -> np.ndarray:
    return np.array([-math.sin(q1), math.cos(q1), 0.0], dtype=float)


def _link_direction_world(q1: float, pitch_angle: float) -> np.ndarray:
    radial = np.array([math.cos(q1), math.sin(q1), 0.0], dtype=float)
    return math.cos(pitch_angle) * radial + math.sin(pitch_angle) * np.array(
        [0.0, 0.0, 1.0], dtype=float
    )


def _point_jacobian(
    point: np.ndarray,
    q: np.ndarray,
    link_lengths_mm: tuple[float, float],
    base_height_mm: float,
) -> tuple[np.ndarray, np.ndarray]:
    q1, _, _ = q
    points = forward_kinematics(
        radians_to_degrees(q),
        link_lengths=link_lengths_mm,
        base_height=base_height_mm,
        degrees=True,
    )
    shoulder = np.array(points[1], dtype=float) / 1000.0
    elbow = np.array(points[2], dtype=float) / 1000.0

    z_axis = np.array([0.0, 0.0, 1.0], dtype=float)
    pitch_axis = _pitch_axis_world(q1)

    origin_1 = np.array([0.0, 0.0, 0.0], dtype=float)
    origin_2 = shoulder
    origin_3 = elbow

    Jv = np.column_stack(
        [
            np.cross(z_axis, point - origin_1),
            np.cross(pitch_axis, point - origin_2),
            np.cross(pitch_axis, point - origin_3),
        ]
    )
    Jw = np.column_stack([z_axis, pitch_axis, pitch_axis])
    return Jv, Jw


def _com_positions_m(
    q: np.ndarray,
    link_lengths_mm: tuple[float, float],
    base_height_mm: float,
) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    q1, q2, q3 = q
    l1_m = link_lengths_mm[0] / 1000.0
    l2_m = link_lengths_mm[1] / 1000.0
    base_height_m = base_height_mm / 1000.0

    radial_dir = np.array([math.cos(q1), math.sin(q1), 0.0], dtype=float)
    z_dir = np.array([0.0, 0.0, 1.0], dtype=float)

    shoulder = np.array([0.0, 0.0, base_height_m], dtype=float)
    link2_dir = _link_direction_world(q1, q2)
    link3_dir = _link_direction_world(q1, q2 + q3)

    com2 = shoulder + 0.5 * l1_m * link2_dir
    elbow = shoulder + l1_m * link2_dir
    com3 = elbow + 0.5 * l2_m * link3_dir
    end_effector = elbow + l2_m * link3_dir
    return com2, com3, end_effector


def _link_inertia_world(mass_kg: float, length_m: float, axis_world: np.ndarray) -> np.ndarray:
    i_axial = 0.5 * mass_kg * LINK_RADIUS_M**2
    i_transverse = (1.0 / 12.0) * mass_kg * (3.0 * LINK_RADIUS_M**2 + length_m**2)
    axis_unit = axis_world / np.linalg.norm(axis_world)
    return i_transverse * np.eye(3) + (i_axial - i_transverse) * np.outer(axis_unit, axis_unit)


def mass_matrix(
    q: np.ndarray,
    link_lengths_mm: tuple[float, float],
    base_height_mm: float,
) -> np.ndarray:
    l1_m = link_lengths_mm[0] / 1000.0
    l2_m = link_lengths_mm[1] / 1000.0
    q1, q2, q3 = q

    com2, com3, end_effector = _com_positions_m(q, link_lengths_mm, base_height_mm)
    link2_dir = _link_direction_world(q1, q2)
    link3_dir = _link_direction_world(q1, q2 + q3)

    Jv2, Jw2 = _point_jacobian(com2, q, link_lengths_mm, base_height_mm)
    Jv3, Jw3 = _point_jacobian(com3, q, link_lengths_mm, base_height_mm)
    Jvp, _ = _point_jacobian(end_effector, q, link_lengths_mm, base_height_mm)

    I2 = _link_inertia_world(LINK_2_MASS_KG, l1_m, link2_dir)
    I3 = _link_inertia_world(LINK_3_MASS_KG, l2_m, link3_dir)

    M = np.zeros((3, 3), dtype=float)
    M += LINK_2_MASS_KG * (Jv2.T @ Jv2) + Jw2.T @ I2 @ Jw2
    M += LINK_3_MASS_KG * (Jv3.T @ Jv3) + Jw3.T @ I3 @ Jw3
    M += PAYLOAD_MASS_KG * (Jvp.T @ Jvp)
    return M


def gravity_vector(
    q: np.ndarray,
    link_lengths_mm: tuple[float, float],
    base_height_mm: float,
) -> np.ndarray:
    gravity_force = np.array([0.0, 0.0, GRAVITY_M_S2], dtype=float)
    com2, com3, end_effector = _com_positions_m(q, link_lengths_mm, base_height_mm)
    Jv2, _ = _point_jacobian(com2, q, link_lengths_mm, base_height_mm)
    Jv3, _ = _point_jacobian(com3, q, link_lengths_mm, base_height_mm)
    Jvp, _ = _point_jacobian(end_effector, q, link_lengths_mm, base_height_mm)

    G = np.zeros(3, dtype=float)
    G += LINK_2_MASS_KG * (Jv2.T @ gravity_force)
    G += LINK_3_MASS_KG * (Jv3.T @ gravity_force)
    G += PAYLOAD_MASS_KG * (Jvp.T @ gravity_force)
    return G


def coriolis_vector(
    q: np.ndarray,
    qdot: np.ndarray,
    link_lengths_mm: tuple[float, float],
    base_height_mm: float,
    epsilon: float = 1e-6,
) -> np.ndarray:
    M = mass_matrix(q, link_lengths_mm, base_height_mm)
    dM = np.zeros((3, 3, 3), dtype=float)
    for k in range(3):
        dq = np.zeros(3, dtype=float)
        dq[k] = epsilon
        mp = mass_matrix(q + dq, link_lengths_mm, base_height_mm)
        mm = mass_matrix(q - dq, link_lengths_mm, base_height_mm)
        dM[:, :, k] = (mp - mm) / (2.0 * epsilon)

    C = np.zeros(3, dtype=float)
    for i in range(3):
        total = 0.0
        for j in range(3):
            for k in range(3):
                christoffel = 0.5 * (
                    dM[i, j, k] + dM[i, k, j] - dM[j, k, i]
                )
                total += christoffel * qdot[j] * qdot[k]
        C[i] = total
    return C


def inverse_dynamics(
    q: np.ndarray,
    qdot: np.ndarray,
    qddot: np.ndarray,
    link_lengths_mm: tuple[float, float],
    base_height_mm: float,
) -> np.ndarray:
    M = mass_matrix(q, link_lengths_mm, base_height_mm)
    C = coriolis_vector(q, qdot, link_lengths_mm, base_height_mm)
    G = gravity_vector(q, link_lengths_mm, base_height_mm)
    return M @ qddot + C + G


class JacobianPickPlaceController:
    def __init__(
        self,
        trajectory: PickAndPlaceTrajectory,
        link_lengths: tuple[float, float] = LINK_LENGTHS,
        base_height: float = BASE_LENGTH_MM,
        interval_ms: int = 20,
    ) -> None:
        self.trajectory = trajectory
        self.link_lengths = link_lengths
        self.base_height = base_height
        self.nominal_dt = interval_ms / 1000.0

        self.max_joint_velocity = np.radians([85.0, 90.0, 110.0])
        self.max_joint_acceleration = np.radians([240.0, 260.0, 320.0])
        self.kp_cartesian = 2.8

        initial_target, _ = self.trajectory.sample(0.0)
        self.current_q = inverse_kinematics_3dof(
            initial_target[0],
            initial_target[1],
            initial_target[2],
            link_lengths=self.link_lengths,
            base_height=self.base_height,
        )
        self.current_qdot = np.zeros(3, dtype=float)
        self.current_qddot = np.zeros(3, dtype=float)
        self.current_points = forward_kinematics(
            radians_to_degrees(self.current_q),
            link_lengths=self.link_lengths,
            base_height=self.base_height,
            degrees=True,
        )
        initial_jacobian_mm = jacobian_3dof(self.current_q, self.link_lengths)
        self.current_ee_velocity = (initial_jacobian_mm / 1000.0) @ self.current_qdot
        self.current_ee_acceleration = np.zeros(3, dtype=float)
        self.current_joint_torque = gravity_vector(
            self.current_q, self.link_lengths, self.base_height
        )
        self.sim_time = 0.0
        self.last_wall_time = time.perf_counter()
        self.wall_time_accumulator = 0.0
        self.max_substeps_per_frame = 5

    def _refresh_cached_points(self) -> None:
        self.current_points = forward_kinematics(
            radians_to_degrees(self.current_q),
            link_lengths=self.link_lengths,
            base_height=self.base_height,
            degrees=True,
        )

    def _integrate_one_fixed_step(self) -> None:
        dt = self.nominal_dt
        self.sim_time += dt

        target_position, target_velocity = self.trajectory.sample(self.sim_time)
        current_position = np.array(self.current_points[-1], dtype=float)
        position_error = target_position - current_position

        commanded_cartesian_velocity = target_velocity + self.kp_cartesian * position_error
        jacobian = jacobian_3dof(self.current_q, self.link_lengths)
        jacobian_pinv = damped_least_squares_inverse(jacobian)
        desired_qdot = jacobian_pinv @ commanded_cartesian_velocity
        desired_qdot = np.clip(desired_qdot, -self.max_joint_velocity, self.max_joint_velocity)

        max_delta_qdot = self.max_joint_acceleration * dt
        qdot_delta = desired_qdot - self.current_qdot
        qdot_delta = np.clip(qdot_delta, -max_delta_qdot, max_delta_qdot)
        previous_qdot = self.current_qdot.copy()
        self.current_qdot = self.current_qdot + qdot_delta
        self.current_qdot = np.clip(
            self.current_qdot, -self.max_joint_velocity, self.max_joint_velocity
        )
        self.current_qddot = (self.current_qdot - previous_qdot) / dt

        self.current_q = self.current_q + self.current_qdot * dt
        self._refresh_cached_points()

        previous_ee_velocity = self.current_ee_velocity.copy()
        updated_jacobian_mm = jacobian_3dof(self.current_q, self.link_lengths)
        self.current_ee_velocity = (updated_jacobian_mm / 1000.0) @ self.current_qdot
        self.current_ee_acceleration = (self.current_ee_velocity - previous_ee_velocity) / dt
        self.current_joint_torque = inverse_dynamics(
            self.current_q,
            self.current_qdot,
            self.current_qddot,
            self.link_lengths,
            self.base_height,
        )

    def _build_frame(self) -> FrameTelemetry:
        return FrameTelemetry(
            joint_angles=radians_to_degrees(self.current_q),
            points=self.current_points.copy(),
            joint_velocity=np.degrees(self.current_qdot).copy(),
            joint_acceleration=np.degrees(self.current_qddot).copy(),
            joint_torque=self.current_joint_torque.copy(),
            end_effector_speed=float(1000.0 * np.linalg.norm(self.current_ee_velocity)),
            end_effector_acceleration=float(1000.0 * np.linalg.norm(self.current_ee_acceleration)),
        )

    def step(self) -> FrameTelemetry:
        now = time.perf_counter()
        elapsed = now - self.last_wall_time
        self.last_wall_time = now
        self.wall_time_accumulator += elapsed

        substeps = 0
        while (
            self.wall_time_accumulator >= self.nominal_dt
            and substeps < self.max_substeps_per_frame
        ):
            self._integrate_one_fixed_step()
            self.wall_time_accumulator -= self.nominal_dt
            substeps += 1

        if substeps == 0:
            self._integrate_one_fixed_step()
        elif substeps == self.max_substeps_per_frame:
            self.wall_time_accumulator = min(self.wall_time_accumulator, self.nominal_dt)

        return self._build_frame()


def main() -> None:
    interval_ms = 10
    controller = JacobianPickPlaceController(
        trajectory=PickAndPlaceTrajectory(),
        link_lengths=LINK_LENGTHS,
        base_height=BASE_LENGTH_MM,
        interval_ms=interval_ms,
    )

    animate_robot_arm(
        angle_source=controller.step,
        interval_ms=interval_ms,
        link_lengths=LINK_LENGTHS,
        base_height=BASE_LENGTH_MM,
        degrees=True,
        trace_end_effector=True,
    )


if __name__ == "__main__":
    main()
