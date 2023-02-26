import numpy as np
import time
import yaml

from data_types import *

EPSILON = 1e-6


class Commander():
    """
    Manages the state machine and high-level control of the robot.
    """

    def __init__(self):
        self.config = yaml.safe_load(open("config.yaml"))

        self.operating_mode = OperatingMode.IDLE
        self.start_time = time.time()
        self.current_time = 0.0
        self.last_transition_time = 0.0

        self.command = np.zeros(1, dtype=command_dtype)

        # Trajectory variables
        self.command_traj_start = np.zeros(1, dtype=command_dtype)
        self.command_traj_end = np.zeros(1, dtype=command_dtype)
        self.command_traj_start_timestamps = np.zeros(
            1, dtype=command_timestamps_dtype)
        self.command_traj_end_timestamps = np.zeros(
            1, dtype=command_timestamps_dtype)

    def update(self, state, user_input):
        """
        Updates the operating_mode and command based on the current state and user input.
        """
        last_time = self.current_time
        self.current_time = time.time() - self.start_time
        dt = self.current_time - last_time
        # dt = 1.0 / self.config["loop_rate_hz"]

        # Update the operating mode.
        if self.operating_mode == OperatingMode.IDLE:
            if not user_input['start_toggle']:
                # If the robot is idle and the user requests an emergency stop, transition to the IDLE operating mode.
                self.idle_transition()
            if user_input['stand_up_toggle']:
                # If the robot is idle and the user wants to stand up, transition to the STAND_UP operating mode.
                self.stand_up_transition()
        elif self.operating_mode == OperatingMode.RUN:
            if not user_input['start_toggle']:
                # If the robot is running and the user requests an emergency stop, transition to the IDLE operating mode.
                self.idle_transition()
            elif not user_input['stand_up_toggle']:
                # If the robot is running and the user wants to sit down, transition to the SIT_DOWN operating mode.
                self.sit_down_transition()
        if self.operating_mode == OperatingMode.STAND_UP:
            if not user_input['start_toggle']:
                # If the robot is standing up and the user requests an emergency stop, transition to the IDLE operating mode.
                self.idle_transition()
            elif self.current_time - self.last_transition_time > self.config['stand_up_duration_s']:
                # If the stand up duration has elapsed, transition to the RUN operating mode.
                self.running_transition()
        if self.operating_mode == OperatingMode.SIT_DOWN:
            if not user_input['start_toggle']:
                # If the robot is sitting down and the user requests an emergency stop, transition to the IDLE operating mode.
                self.idle_transition()
            elif self.current_time - self.last_transition_time > self.config['sit_down_duration_s']:
                # If the sit down duration has elapsed, transition to the IDLE operating mode.
                self.idle_transition()

        if self.operating_mode == OperatingMode.IDLE:
            # If the robot is idle, set the command to zero to indicate that the robot should not move.
            self.command[:] = 0
        elif self.operating_mode == OperatingMode.RUN:
            # If the robot is running, update the commanded velocity trajectory based on the user input.
            desired_x_vel = user_input["x"] * self.config["x_vel_limit_mps"]
            desired_yaw_vel = user_input["yaw"] * \
                self.config["yaw_vel_limit_rps"]

            # Limit the commanded velocity trajectory to the acceleration limits.
            desired_x_vel_acc_limited = np.clip(
                desired_x_vel - self.command_traj_end["odometry_x_vel"], -self.config["x_acc_limit_mps2"] * dt, self.config["x_acc_limit_mps2"] * dt)
            desired_yaw_vel_acc_limited = np.clip(
                desired_yaw_vel - self.command_traj_end["odometry_yaw_vel"], -self.config["yaw_acc_limit_mps2"] * dt, self.config["yaw_acc_limit_mps2"] * dt)
            desired_x_vel = self.command_traj_end["odometry_x_vel"] + \
                desired_x_vel_acc_limited
            desired_yaw_vel = self.command_traj_end["odometry_yaw_vel"] + \
                desired_yaw_vel_acc_limited

            self.command_traj_start["odometry_x_vel"] = desired_x_vel
            self.command_traj_start["odometry_yaw_vel"] = desired_yaw_vel
            self.command_traj_end["odometry_x_vel"] = desired_x_vel
            self.command_traj_end["odometry_yaw_vel"] = desired_yaw_vel
            # Set the commanded positions based on the commanded velocities and timestep.
            self.command_traj_start["odometry_x"] += \
                dt * desired_x_vel
            self.command_traj_start["odometry_yaw"] += \
                dt * desired_yaw_vel
            self.command_traj_end["odometry_x"] += \
                dt * desired_x_vel
            self.command_traj_end["odometry_yaw"] += \
                dt * desired_yaw_vel
            # Set all the odometry timestamps to the current time.
            for key in ["odometry_x", "odometry_yaw", "odometry_x_vel", "odometry_yaw_vel"]:
                self.command_traj_start_timestamps[key] = self.current_time
                self.command_traj_end_timestamps[key] = self.current_time + dt

        if not state["ground_contact_r"] or not state["ground_contact_l"]:
            # If the robot is not on the ground, set the command to zero to indicate that the robot should not move.
            self.zero_x_yaw_vel()
            self.zero_x_yaw()
        # print(state["odometry_x"], state["odometry_x_vel"], self.command["odometry_x"], self.command["odometry_x_vel"])

        # Interpolate the command based on the current time.
        trajectory_proportions = (self.current_time - self.command_traj_start_timestamps.view('f4')) / (
            self.command_traj_end_timestamps.view('f4') - self.command_traj_start_timestamps.view('f4') + EPSILON)
        trajectory_proportions = np.clip(trajectory_proportions, 0, 1)
        self.command = self.command_traj_start.view('f4') + trajectory_proportions * (
            self.command_traj_end.view('f4') - self.command_traj_start.view('f4'))
        # Convert the command to the correct data type.
        self.command = self.command.view(command_dtype)

    def idle_transition(self):
        """
        Transition function for the IDLE operating mode.
        """
        self.last_transition_time = self.current_time
        self.operating_mode = OperatingMode.IDLE
        self.zero_x_yaw_vel()
        self.zero_x_yaw()

    def stand_up_transition(self):
        """
        Transition function for the STAND_UP operating mode.
        """
        self.last_transition_time = self.current_time
        self.operating_mode = OperatingMode.STAND_UP
        self.zero_x_yaw_vel()
        # Set the trajectory start and end points.
        self.command_traj_start["leg_length_r"] = self.config["sitting_leg_length_m"]
        self.command_traj_start["leg_length_l"] = self.config["sitting_leg_length_m"]
        self.command_traj_end["leg_length_r"] = self.config["standing_leg_length_m"]
        self.command_traj_end["leg_length_l"] = self.config["standing_leg_length_m"]
        # Set the trajectory start and end timestamps.
        self.command_traj_start_timestamps["leg_length_r"] = self.current_time
        self.command_traj_start_timestamps["leg_length_l"] = self.current_time
        self.command_traj_end_timestamps["leg_length_r"] = self.current_time + \
            self.config["stand_up_duration_s"]
        self.command_traj_end_timestamps["leg_length_l"] = self.current_time + \
            self.config["stand_up_duration_s"]

    def sit_down_transition(self):
        """
        Transition function for the SIT_DOWN operating mode.
        """
        self.last_transition_time = self.current_time
        self.operating_mode = OperatingMode.SIT_DOWN
        self.zero_x_yaw_vel()
        # Set the trajectory start and end points.
        self.command_traj_start["leg_length_r"] = self.config["standing_leg_length_m"]
        self.command_traj_start["leg_length_l"] = self.config["standing_leg_length_m"]
        self.command_traj_end["leg_length_r"] = self.config["sitting_leg_length_m"]
        self.command_traj_end["leg_length_l"] = self.config["sitting_leg_length_m"]
        # Set the trajectory start and end timestamps.
        self.command_traj_start_timestamps["leg_length_r"] = self.current_time
        self.command_traj_start_timestamps["leg_length_l"] = self.current_time
        self.command_traj_end_timestamps["leg_length_r"] = self.current_time + \
            self.config["sit_down_duration_s"]
        self.command_traj_end_timestamps["leg_length_l"] = self.current_time + \
            self.config["sit_down_duration_s"]

    def running_transition(self):
        """
        Transition function for the RUN operating mode.
        """
        self.last_transition_time = self.current_time
        self.operating_mode = OperatingMode.RUN

    def zero_x_yaw_vel(self):
        # Zero out the x and yaw velocity commands.
        self.command_traj_start["odometry_x_vel"] = 0
        self.command_traj_start["odometry_yaw_vel"] = 0
        self.command_traj_end["odometry_x_vel"] = 0
        self.command_traj_end["odometry_yaw_vel"] = 0
        # Zero out the x and yaw velocity timestamps.
        self.command_traj_start_timestamps["odometry_x_vel"] = self.current_time
        self.command_traj_start_timestamps["odometry_yaw_vel"] = self.current_time
        self.command_traj_end_timestamps["odometry_x_vel"] = self.current_time
        self.command_traj_end_timestamps["odometry_yaw_vel"] = self.current_time

    def zero_x_yaw(self):
        # Zero out the x and yaw commands.
        self.command_traj_start["odometry_x"] = 0
        self.command_traj_start["odometry_yaw"] = 0
        self.command_traj_end["odometry_x"] = 0
        self.command_traj_end["odometry_yaw"] = 0
        # Zero out the x and yaw timestamps.
        self.command_traj_start_timestamps["odometry_x"] = self.current_time
        self.command_traj_start_timestamps["odometry_yaw"] = self.current_time
        self.command_traj_end_timestamps["odometry_x"] = self.current_time
        self.command_traj_end_timestamps["odometry_yaw"] = self.current_time


if __name__ == "__main__":
    """
    Test the commander by instantiating it and running it with actual user input.
    """
    from user_input_manager import UserInputManager
    from data_types import state_dtype

    # Create the commander.
    commander = Commander()
    # Create the user input manager.
    user_input_manager = UserInputManager()
    # Create a fake state.
    state = np.zeros(1, dtype=state_dtype)

    # Loop forever and print the operating mode and commanded state.
    while True:
        user_input_manager.update()
        user_input = user_input_manager.user_input
        commander.update(state, user_input)
        print("Operating mode: {}".format(commander.operating_mode))
        print("Commanded state: {}".format(commander.command))
        print()
        time.sleep(0.2)
