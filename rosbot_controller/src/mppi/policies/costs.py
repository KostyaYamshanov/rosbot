import numpy as np
from numba import njit


def nearest_cost(state, reference_trajectory, reference_intervals, desired_v):
    """Cost according to k nearest points.

    Args:
        state: np.ndarray of shape [batch_size, time_steps, 8] where 3 for x, y, yaw, v, w, v_control, w_control, dts
        ref_traj: np.array of shape [ref_traj_size, 3] where 3 for x, y, yaw
        traj_lookahead: int
        goal_idx: int
        goals_interval: float
        desired_v: float

    Return:
        costs: np.array of shape [batch_size]
    """
    v = state[:, :, 3]

    costs = lin_vel_cost(v, desired_v)
    costs += nearest_cost_for_k_ellements(state, reference_trajectory, 3)

    return costs


def nearest_cost_for_k_ellements(state, reference_trajectory, k_idx):
    x_dists = state[:, :, :1] - reference_trajectory[:, 0]
    y_dists = state[:, :, 1:2] - reference_trajectory[:, 1]
    dists = x_dists ** 2 + y_dists ** 2

    k = min(k_idx, len(reference_trajectory))
    dists = np.partition(dists, k - 1, axis=2)[:, :, :k] * np.arange(1, k + 1)

    return dists.min(2).sum(1)


@njit
def triangle_cost(state, reference_trajectory, reference_intervals, desired_v):
    """Cost according to nearest segment.

    Args:
        state: np.ndarray of shape [batch_size, time_steps, 8] where 3 for x, y, yaw, v, w, v_control, w_control, dts
        ref_traj: np.array of shape [ref_traj_size, 3] where 3 for x, y, yaw
        desired_v: float

    Return:
        costs: np.array of shape [batch_size]
    """
    v = state[:, :, 3]
    costs = lin_vel_cost(v, desired_v)
    costs += triangle_cost_segments(state, reference_trajectory, reference_intervals)

    return costs


@njit
def triangle_cost_segments(state, reference_trajectory, reference_intervals):
    x_dists = state[:, :, :1] - reference_trajectory[:, 0]
    y_dists = state[:, :, 1:2] - reference_trajectory[:, 1]
    dists = np.sqrt(x_dists ** 2 + y_dists ** 2)

    if len(reference_trajectory) == 1:
        return dists.sum(2).sum(1)

    first_sides = dists[:, :, :-1]
    second_sides = dists[:, :, 1:]
    opposite_sides = reference_intervals

    first_obtuse_mask = is_angle_obtuse(first_sides, second_sides, opposite_sides)
    second_obtuse_mask = is_angle_obtuse(second_sides, first_sides, opposite_sides)

    cost = np.zeros(shape=(dists.shape[0]))
    for i in range(dists.shape[0]):
        for j in range(dists.shape[1]):
            dists_to_segments = np.empty(len(reference_intervals))
            for k in range(len(reference_intervals)):
                first_side = first_sides[i, j, k]
                second_side = second_sides[i, j, k]
                opposite_side = opposite_sides[k]
                if is_angle_obtuse(first_side, second_side, opposite_side):
                    dists_to_segments[k] = first_side
                elif is_angle_obtuse(second_side, first_side, opposite_side):
                    dists_to_segments[k] = second_side
                else:
                    dists_to_segments[k] = heron(
                        opposite_side,
                        first_side,
                        second_side
                    )
            cost[i] += dists_to_segments.min()

    return cost


@njit
def is_angle_obtuse(opposite_side, b, c):
    return opposite_side ** 2 > (b ** 2 + c ** 2)


@njit
def heron(opposite_side, b, c):
    eps = 0.00001
    if abs(opposite_side) < eps:
        return min(b, c)

    p = (opposite_side + b + c) / 2.0
    h = 2.0 / opposite_side * np.sqrt(p * (p - opposite_side) * (p - b) * (p - c))
    return h


@njit
def lin_vel_cost(v, desired_v):
    DESIRED_V_WEIGHT = 2.0
    v_costs = DESIRED_V_WEIGHT * ((v - desired_v)**2).sum(1)
    return v_costs
