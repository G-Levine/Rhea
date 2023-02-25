import time
import numpy as np
import os
from os.path import dirname, join, abspath
import pinocchio as pin
from pinocchio.visualize import MeshcatVisualizer
import meshcat.geometry as g

"""
Joint names:
    0 universe
    1 root_joint
    2 leg_1_l
    3 leg_2_l
    4 wheel_l
    5 leg_1_r
    6 leg_2_r
    7 wheel_r

Configuration vector:
    0 x
    1 y
    2 z
    3 qx
    4 qy
    5 qz
    6 qw
    7 leg_1_l
    8 leg_2_l
    9 wheel_l_cos
    10 wheel_l_sin
    11 leg_1_r
    12 leg_2_r
    13 wheel_r_cos
    14 wheel_r_sin
"""


class TelemetryManager:
    """
    Visualize the Rhea robot in Meshcat.
    Allows updating the robot state in real-time.
    """

    def __init__(self, urdf_model_path="urdf/rhea.urdf", mesh_dir="meshes"):
        self.model, self.collision_model, self.visual_model = pin.buildModelsFromUrdf(
            urdf_model_path, mesh_dir, pin.JointModelFreeFlyer()
        )
        self.vis = MeshcatVisualizer(
            self.model, self.collision_model, self.visual_model)
        self.vis.initViewer(open=False)
        self.vis.loadViewerModel()
        self.q = pin.neutral(self.model)

        self.joint_indices = {
            "x": 0,
            "y": 1,
            "z": 2,
            "qx": 3,
            "qy": 4,
            "qz": 5,
            "qw": 6,
            "leg_1_l": 7,
            "leg_2_l": 8,
            "wheel_l_cos": 9,
            "wheel_l_sin": 10,
            "leg_1_r": 11,
            "leg_2_r": 12,
            "wheel_r_cos": 13,
            "wheel_r_sin": 14,
        }

    def update(self, state):
        # IMU state
        # self.q[self.joint_indices["x"]] = state.get("x", 0)
        # self.q[self.joint_indices["y"]] = state.get("y", 0)
        # self.q[self.joint_indices["z"]] = state.get("z", 0)

        qx, qy, qz, qw = self.roll_pitch_yaw_to_quaternion(
            state["roll"], state["pitch"], 0)
        self.q[self.joint_indices["qx"]] = qx
        self.q[self.joint_indices["qy"]] = qy
        self.q[self.joint_indices["qz"]] = qz
        self.q[self.joint_indices["qw"]] = qw

        # Joint angles for the actuated joints
        self.q[self.joint_indices["leg_1_l"]] = state["leg_1_l_pos"]
        self.q[self.joint_indices["leg_1_r"]] = state["leg_1_r_pos"]

        # Joint angles for the passive joints
        self.q[self.joint_indices["leg_2_l"]] = -2 * state["leg_1_l_pos"]
        self.q[self.joint_indices["leg_2_r"]] = -2 * state["leg_1_r_pos"]

        # Wheels are represented as a sin/cos pair
        self.q[self.joint_indices["wheel_l_cos"]
               ] = np.cos(state["wheel_l_pos"])
        self.q[self.joint_indices["wheel_l_sin"]
               ] = np.sin(state["wheel_l_pos"])
        self.q[self.joint_indices["wheel_r_cos"]
               ] = np.cos(state["wheel_r_pos"])
        self.q[self.joint_indices["wheel_r_sin"]
               ] = np.sin(state["wheel_r_pos"])

        # Display the robot
        self.vis.display(self.q)

    def roll_pitch_yaw_to_quaternion(self, roll, pitch, yaw):
        """
        Convert roll, pitch, and yaw angles to a quaternion.
        """
        qx = np.sin(roll / 2) * np.cos(pitch / 2) * np.cos(yaw / 2) - \
            np.cos(roll / 2) * np.sin(pitch / 2) * np.sin(yaw / 2)
        qy = np.cos(roll / 2) * np.sin(pitch / 2) * np.cos(yaw / 2) + \
            np.sin(roll / 2) * np.cos(pitch / 2) * np.sin(yaw / 2)
        qz = np.cos(roll / 2) * np.cos(pitch / 2) * np.sin(yaw / 2) - \
            np.sin(roll / 2) * np.sin(pitch / 2) * np.cos(yaw / 2)
        qw = np.cos(roll / 2) * np.cos(pitch / 2) * np.cos(yaw / 2) + \
            np.sin(roll / 2) * np.sin(pitch / 2) * np.sin(yaw / 2)
        return qx, qy, qz, qw


if __name__ == "__main__":
    rhea_vis = TelemetryManager()

    # Loop to update the robot state
    while True:
        # Update the robot state here
        observation = {
            "leg_1_l": 0.5,
        }
        rhea_vis.update(observation)
        time.sleep(0.1)
