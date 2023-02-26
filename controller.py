import numpy as np
import yaml

from data_types import *

EPSILON = 1e-6


class Controller():
    """
    Controller class.
    """

    def __init__(self):
        self.config = yaml.safe_load(open("config.yaml"))
        self.action = np.zeros(1, dtype=action_dtype)

    def update(self, state, command):
        """
        Updates the action based on the current state and command.
        """
        # If the robot is in the IDLE operating mode (command is the zero vector), set the action to zero.
        if np.all(command.view('f4') == 0.0):
            self.action[:] = 0
            return

        """
        Leg control.
        """
        roll_compensation = np.clip(state["roll"] * (self.config["wheelbase_m"] / 2.0), -0.1, 0.1)
        leg_angle_target_r = - \
            np.arccos((command["leg_length_r"] + roll_compensation) /
                      (2 * self.config["leg_link_m"]))
        leg_angle_target_l = np.arccos(
            (command["leg_length_l"] - roll_compensation) / (2 * self.config["leg_link_m"]))

        self.action["desired_leg_1_r_pos"] = leg_angle_target_r
        self.action["desired_leg_1_l_pos"] = leg_angle_target_l

        self.action["desired_leg_1_r_kp"] = self.config["leg_kp"]
        self.action["desired_leg_1_l_kp"] = self.config["leg_kp"]

        self.action["desired_leg_1_r_kd"] = self.config["leg_kd"]
        self.action["desired_leg_1_l_kd"] = self.config["leg_kd"]

        """
        Wheel control.
        """
        yaw_pos_error = command["odometry_yaw"] - state["odometry_yaw"]
        yaw_vel_error = command["odometry_yaw_vel"] - state["odometry_yaw_vel"]

        leg_length_r = self.config["leg_link_m"] * \
            np.cos(state["leg_1_r_pos"]) * 2
        leg_length_l = self.config["leg_link_m"] * \
            np.cos(state["leg_1_l_pos"]) * 2
        leg_length = (leg_length_r + leg_length_l) / 2

        leg_r_force = min(0, -state["leg_1_r_effort"]) / (self.config["leg_link_m"] * np.sin(state["leg_1_r_pos"] + EPSILON))
        leg_l_force = min(0, state["leg_1_l_effort"]) / (self.config["leg_link_m"] * np.sin(-state["leg_1_l_pos"] + EPSILON))

        if not state["ground_contact_r"] or not state["ground_contact_l"]:
            self.action["desired_wheel_r_effort"] = 0
            self.action["desired_wheel_l_effort"] = 0
            return

        friction_limited_wheel_effort = max(leg_r_force, leg_l_force) * self.config["wheel_diam_m"] / 2 * self.config["friction_coeff"]
        k_x, k_pitch, k_x_vel, k_pitch_vel = self.get_balance_gains(leg_length)

        balance_effort = k_x * (command["odometry_x"] - state["odometry_x"]) + k_pitch * (-state["pitch"]) + k_x_vel * (
            command["odometry_x_vel"] - state["odometry_x_vel"]) + k_pitch_vel * (-state["pitch_vel"])
        yaw_effort = self.config["yaw_kp"] * yaw_pos_error + \
            self.config["yaw_kd"] * yaw_vel_error

        wheel_r_effort = balance_effort + yaw_effort
        wheel_l_effort = -balance_effort + yaw_effort

        wheel_r_effort = np.clip(wheel_r_effort, -friction_limited_wheel_effort, friction_limited_wheel_effort)
        wheel_l_effort = np.clip(wheel_l_effort, -friction_limited_wheel_effort, friction_limited_wheel_effort)

        self.action["desired_wheel_r_effort"] = wheel_r_effort
        self.action["desired_wheel_l_effort"] = wheel_l_effort

    def get_balance_gains(self, leg_length):
        """
        Returns the balance gains based on the current state.
        """
        k_x = self.config["k_x"]
        k_pitch = self.config["k_pitch"]
        k_x_vel = self.config["k_x_vel"]
        k_pitch_vel = self.config["k_pitch_vel"]
        return k_x, k_pitch, k_x_vel, k_pitch_vel
