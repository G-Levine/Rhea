import enum


class OperatingMode(enum.Enum):
    """Operating modes of the robot."""
    IDLE = 0
    STAND_UP = 1
    RUN = 2
    SIT_DOWN = 3


"""Observations of the robot."""
observation_dtype = [
    ('roll', 'f4'),
    ('pitch', 'f4'),
    ('roll_vel', 'f4'),
    ('pitch_vel', 'f4'),
    ('leg_1_r_pos', 'f4'),
    ('wheel_r_pos', 'f4'),
    ('leg_1_l_pos', 'f4'),
    ('wheel_l_pos', 'f4'),
    ('leg_1_r_vel', 'f4'),
    ('wheel_r_vel', 'f4'),
    ('leg_1_l_vel', 'f4'),
    ('wheel_l_vel', 'f4'),
    ('leg_1_r_effort', 'f4'),
    ('wheel_r_effort', 'f4'),
    ('leg_1_l_effort', 'f4'),
    ('wheel_l_effort', 'f4'),
]


"""State of the robot."""
state_dtype = [
    # State variables directly observed by the robot.
    ('roll', 'f4'),
    ('pitch', 'f4'),
    ('roll_vel', 'f4'),
    ('pitch_vel', 'f4'),
    ('leg_1_r_pos', 'f4'),
    ('wheel_r_pos', 'f4'),
    ('leg_1_l_pos', 'f4'),
    ('wheel_l_pos', 'f4'),
    ('leg_1_r_vel', 'f4'),
    ('wheel_r_vel', 'f4'),
    ('leg_1_l_vel', 'f4'),
    ('wheel_l_vel', 'f4'),
    ('leg_1_r_effort', 'f4'),
    ('wheel_r_effort', 'f4'),
    ('leg_1_l_effort', 'f4'),
    ('wheel_l_effort', 'f4'),
    # State variables estimated by the robot.
    ('com_x', 'f4'),
    ('com_z', 'f4'),
    ('odometry_x', 'f4'),
    ('odometry_yaw', 'f4'),
    ('odometry_x_vel', 'f4'),
    ('odometry_yaw_vel', 'f4'),
    ('ground_contact_r', 'b1'),
    ('ground_contact_l', 'b1'),
]


"""Actions given to the hardware interface."""
action_dtype = [
    ('desired_leg_1_r_pos', 'f4'),
    ('desired_wheel_r_pos', 'f4'),
    ('desired_leg_1_l_pos', 'f4'),
    ('desired_wheel_l_pos', 'f4'),
    ('desired_leg_1_r_vel', 'f4'),
    ('desired_wheel_r_vel', 'f4'),
    ('desired_leg_1_l_vel', 'f4'),
    ('desired_wheel_l_vel', 'f4'),
    ('desired_leg_1_r_effort', 'f4'),
    ('desired_wheel_r_effort', 'f4'),
    ('desired_leg_1_l_effort', 'f4'),
    ('desired_wheel_l_effort', 'f4'),
    ('desired_leg_1_r_kp', 'f4'),
    ('desired_wheel_r_kp', 'f4'),
    ('desired_leg_1_l_kp', 'f4'),
    ('desired_wheel_l_kp', 'f4'),
    ('desired_leg_1_r_kd', 'f4'),
    ('desired_wheel_r_kd', 'f4'),
    ('desired_leg_1_l_kd', 'f4'),
    ('desired_wheel_l_kd', 'f4'),
]


"""User input to the robot."""
user_input_dtype = [
    ('start_toggle', 'b1'),
    ('stand_up_toggle', 'b1'),
    ('x', 'f4'),
    ('yaw', 'f4'),
]


"""Commands given to the controller."""
command_dtype = [
    ('odometry_x', 'f4'),
    ('odometry_yaw', 'f4'),
    ('odometry_x_vel', 'f4'),
    ('odometry_yaw_vel', 'f4'),
    ('leg_length_r', 'f4'),
    ('leg_length_l', 'f4'),
]

"""Command timestamps used to construct a trajectory."""
command_timestamps_dtype = [
    ('odometry_x', 'f4'),
    ('odometry_yaw', 'f4'),
    ('odometry_x_vel', 'f4'),
    ('odometry_yaw_vel', 'f4'),
    ('leg_length_r', 'f4'),
    ('leg_length_l', 'f4'),
]
