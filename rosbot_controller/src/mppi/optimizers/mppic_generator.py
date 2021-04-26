from utils.visualizations import visualize_trajs, MarkerArray
from utils.geometry import quaternion_to_euler
from utils.dtypes import State, Control

import rospy
from time import perf_counter
from typing import Callable, Type
import numpy as np
import sys

sys.path.append("..")


class MPPICGenerator():
    def __init__(self, model):
        self.state = State()
        self.freq = int(rospy.get_param("~mppic/mppi_freq", 30))
        self.dt = 1.0 / self.freq
        self.time_steps = int(rospy.get_param("~mppic/time_steps", 50))
        self.batch_size = int(rospy.get_param("~mppic/batch_size", 100))

        self._model = model

        self._v_std = rospy.get_param("~mppic/v_std", 0.1)
        self._w_std = rospy.get_param("~mppic/w_std", 0.1)

        self._limit_v = rospy.get_param("~mppic/limit_v", 0.5)
        self._limit_w = rospy.get_param("~mppic/limit_w", 0.7)

        # 5 for v, w, control_dim and dt
        self._batch_of_seqs = np.zeros(shape=(self.batch_size, self.time_steps, 5))
        self._batch_of_seqs[:, :, 4] = self.dt
        self._curr_control_seq = np.zeros(shape=(self.time_steps, 2))

    def set_control_seq(self, control_seq):
        self._curr_control_seq = control_seq

    def get_velocities_batch(self):
        return self._batch_of_seqs[:, :, :2]

    def get_controls_batch(self):
        return self._batch_of_seqs[:, :, 2:4]

    def get_control(self, offset: int):
        offset = min(offset, self.time_steps - 1)
        v_best = self._curr_control_seq[0 + offset, 0]
        w_best = self._curr_control_seq[0 + offset, 1]
        return Control(v_best, w_best)

    def generate_trajectories(self):
        self._update_batch_of_seqs()
        trajectories = self._propagete_trajectories()
        trajectories = self._insert_velocities(trajectories)
        return trajectories

    def displace_controls(self, offset: int):
        if offset == 0:
            return

        control_cropped = self._curr_control_seq[offset:]
        end_part = np.array([self._curr_control_seq[-1]] * offset)
        self._curr_control_seq = np.concatenate([control_cropped, end_part], axis=0)

    def _insert_velocities(self, trajectories):
        return np.concatenate([trajectories, self.get_velocities_batch(), self.get_controls_batch()], axis=2)

    def _update_batch_of_seqs(self):
        noises = self._generate_noises()
        self._batch_of_seqs[:, 0, 0] = self.state.v
        self._batch_of_seqs[:, 0, 1] = self.state.w
        self._batch_of_seqs[:, :, 2:4] = self._curr_control_seq[None] + noises
        self._batch_of_seqs[:, :, 2:3] = np.clip(
            self._batch_of_seqs[:, :, 2:3], -self._limit_v, self._limit_v
        )
        self._batch_of_seqs[:, :, 3:4] = np.clip(
            self._batch_of_seqs[:, :, 3:4], -self._limit_w, self._limit_w
        )
        self._update_velocities()

    def _generate_noises(self):
        v_noises = np.random.normal(0.0, self._v_std, size=(self.batch_size, self.time_steps, 1))
        w_noises = np.random.normal(0.0, self._w_std, size=(self.batch_size, self.time_steps, 1))
        noises = np.concatenate([v_noises, w_noises], axis=2)

        return noises

    def _update_velocities(self):
        for t_step in range(self.time_steps - 1):
            curr_batch = self._batch_of_seqs[:, t_step].astype(np.float32)
            curr_predicted = self._model(curr_batch)
            self._batch_of_seqs[:, t_step + 1, :2] = curr_predicted

    # todo separate methods for this
    def _propagete_trajectories(self):
        """Propagetes trajectories using control matrix velocities and current state.

        Return:
            trajectory points - np.array of shape [batch_size, time_steps, 3] where 3 is for x, y, yaw respectively
        """
        v, w = self._batch_of_seqs[:, :, 0], self._batch_of_seqs[:, :, 1]
        current_yaw = self.state.yaw
        yaw = np.cumsum(w * self.dt, axis=1)
        yaw += current_yaw - yaw[:, :1]
        v_x = v * np.cos(yaw)
        v_y = v * np.sin(yaw)
        x = np.cumsum(v_x * self.dt, axis=1)
        y = np.cumsum(v_y * self.dt, axis=1)
        x += self.state.x - x[:, :1]
        y += self.state.y - y[:, :1]

        trajectory_points = np.concatenate(
            [
                x[:, :, np.newaxis],
                y[:, :, np.newaxis],
                yaw[:, :, np.newaxis],
            ],
            axis=2,
        )

        return trajectory_points
