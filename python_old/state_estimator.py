import numpy as np
import yaml

from data_types import *

EPSILON = 1e-6

class StateEstimator():
    """
    StateEstimator class.
    """
    def __init__(self):
        self.config = yaml.safe_load(open("config.yaml"))
        self.state = np.zeros(1, dtype=state_dtype)
    
    def update(self, observation):
        """
        Updates the state based on the current observation.
        """
        for key in observation.dtype.names:
            # Copy the observation into the state.
            self.state[key] = observation[key]
        
        # Update the center of mass position.
        self.state["com_x"] = self.config["default_com_x"]
        self.state["com_z"] = self.config["leg_link_m"] * \
            np.cos(self.state["leg_1_r_pos"]) + self.config["leg_link_m"] * \
            np.cos(self.state["leg_1_l_pos"])
        # Update the pitch estimate accounting for the center of mass displacement.
        self.state["pitch"] = observation["pitch"] + np.arctan(self.state["com_x"], self.state["com_z"])

        # Update the odometry values.
        leg_r_force = min(0, -observation["leg_1_r_effort"]) / (self.config["leg_link_m"] * np.sin(observation["leg_1_r_pos"] + EPSILON))
        leg_l_force = min(0, observation["leg_1_l_effort"]) / (self.config["leg_link_m"] * np.sin(-observation["leg_1_l_pos"] + EPSILON))

        if leg_r_force < self.config["contact_threshold_n"] or leg_l_force < self.config["contact_threshold_n"]:
            # Zero the odometry values if the robot is not in contact with the ground.
            self.state["odometry_x"] = 0
            self.state["odometry_yaw"] = 0
            self.state["odometry_x_vel"] = 0
            self.state["odometry_yaw_vel"] = 0
            self.state["ground_contact_r"] = False
            self.state["ground_contact_l"] = False
        else:
            x_vel, yaw_vel = self.wheel_r_l_to_x_yaw(observation["wheel_r_vel"], observation["wheel_l_vel"])
            self.state["ground_contact_r"] = True
            self.state["ground_contact_l"] = True
            self.state["odometry_x"] += x_vel / self.config["loop_rate_hz"]
            self.state["odometry_yaw"] += yaw_vel / self.config["loop_rate_hz"]
            self.state["odometry_x_vel"] = x_vel
            self.state["odometry_yaw_vel"] = yaw_vel


    def wheel_r_l_to_x_yaw(self, wheel_r_val, wheel_l_val):
        """
        Converts values in wheel coordinates to x and yaw coordinates.
        """
        wheel_r_val_x = self.wheel_to_x(wheel_r_val)
        wheel_l_val_x = self.wheel_to_x(wheel_l_val)
        x = (wheel_l_val_x - wheel_r_val_x) / 2
        yaw = (wheel_r_val_x + wheel_l_val_x) / 2 / (self.config["wheelbase_m"] / 2)
        return x, yaw
    
    def wheel_to_x(self, wheel_val):
        """
        Converts a value in wheel coordinates to x coordinates.
        """
        return wheel_val * (self.config["wheel_diam_m"] * np.pi)

    def x_to_wheel(self, x_val):
        """
        Converts a value in x coordinates to wheel coordinates.
        """
        return x_val / (self.config["wheel_diam_m"] * np.pi)