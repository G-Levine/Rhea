import os
import numpy as np
import pybullet as p
import pybullet_data
import time
import yaml

from data_types import *


class HardwareInterfaceSimulator():
    """
    Class that runs a Mujoco simulation and provides an interface to the simulated robot.
    """

    def __init__(self):
        self.observation = np.zeros(1, dtype=observation_dtype)

        self.control_rate_hz = yaml.safe_load(open("config.yaml"))["control_rate_hz"]
        self.config = yaml.safe_load(open("config.yaml"))["actuators"]

        p.connect(p.GUI)
        p.configureDebugVisualizer(p.COV_ENABLE_GUI, 0)
        p.setGravity(0, 0, -9.81)
        p.setTimeStep(1.0 / self.control_rate_hz)
        p.setAdditionalSearchPath(pybullet_data.getDataPath())
        self.plane = p.loadURDF("plane.urdf")
        p.setAdditionalSearchPath(os.path.join('urdf'))

        baseOrientation = p.getQuaternionFromEuler([0, -0.2, 0])
        # self.robot = p.loadURDF("rhea.urdf", [0, 0, 1.3], baseOrientation, useFixedBase=True)
        self.robot = p.loadURDF("rhea.urdf", [0, 0, 0.3], useFixedBase=False)

        # # Print joint names and indices.
        # for i in range(p.getNumJoints(self.robot)):
        #     print(p.getJointInfo(self.robot, i))

        # Reset right leg to the sitting position.
        self.leg_1_l_default = self.config["leg_1_l"]["zero_offset_deg"] * np.pi / 180.0
        p.resetJointState(self.robot, 0, self.leg_1_l_default)
        p.resetJointState(self.robot, 1, -2 * self.leg_1_l_default)
        # Reset left leg to the sitting position.
        self.leg_1_r_default = self.config["leg_1_r"]["zero_offset_deg"] * np.pi / 180.0
        p.resetJointState(self.robot, 3, self.leg_1_r_default)
        p.resetJointState(self.robot, 4, -2 * self.leg_1_r_default)
        # Set up mimic joints.
        leg_l_2_constraint = p.createConstraint(self.robot, 0, self.robot, 1, p.JOINT_GEAR, jointAxis=[
                                                0, 0, 1], parentFramePosition=[0, 0, 0], childFramePosition=[0, 0, 0])
        p.changeConstraint(leg_l_2_constraint, gearRatio=0.5,
                           maxForce=10000, erp=0.2)
        leg_r_2_constraint = p.createConstraint(self.robot, 3, self.robot, 4, p.JOINT_GEAR, jointAxis=[
                                                0, 0, 1], parentFramePosition=[0, 0, 0], childFramePosition=[0, 0, 0])
        p.changeConstraint(leg_r_2_constraint, gearRatio=0.5,
                           maxForce=10000, erp=0.2)
        # Disable damping.
        p.setJointMotorControlArray(
            self.robot, [0, 1, 2, 3, 4, 5], p.VELOCITY_CONTROL, forces=[0, 0, 0, 0, 0, 0])

    def update(self, action):
        # Set the joint positions.
        p.setJointMotorControl2(self.robot, 0, p.POSITION_CONTROL,
                                targetPosition=action["desired_leg_1_l_pos"], force=20 * (action["desired_leg_1_l_kp"] > 0))
        p.setJointMotorControl2(self.robot, 3, p.POSITION_CONTROL,
                                targetPosition=action["desired_leg_1_r_pos"], force=20 * (action["desired_leg_1_r_kp"] > 0))
        # Set the wheel efforts.
        p.setJointMotorControl2(
            self.robot, 2, p.TORQUE_CONTROL, force=action["desired_wheel_l_effort"])
        p.setJointMotorControl2(
            self.robot, 5, p.TORQUE_CONTROL, force=action["desired_wheel_r_effort"])
        p.stepSimulation()

        attitude = p.getEulerFromQuaternion(
            p.getBasePositionAndOrientation(self.robot)[1])
        self.observation["roll"] = attitude[0]
        self.observation["pitch"] = attitude[1]

        velocity, angular_velocity = p.getBaseVelocity(self.robot)
        self.observation["roll_vel"] = angular_velocity[0]
        self.observation["pitch_vel"] = angular_velocity[1]

        self.observation["leg_1_l_pos"] = p.getJointState(self.robot, 0)[0]
        self.observation["leg_1_r_pos"] = p.getJointState(self.robot, 3)[0]
        self.observation["wheel_l_pos"] = p.getJointState(self.robot, 2)[0]
        self.observation["wheel_r_pos"] = p.getJointState(self.robot, 5)[0]

        self.observation["leg_1_l_vel"] = p.getJointState(self.robot, 0)[1]
        self.observation["leg_1_r_vel"] = p.getJointState(self.robot, 3)[1]
        self.observation["wheel_l_vel"] = p.getJointState(self.robot, 2)[1]
        self.observation["wheel_r_vel"] = p.getJointState(self.robot, 5)[1]

        self.observation["leg_1_l_effort"] = p.getJointState(self.robot, 0)[3]
        self.observation["leg_1_r_effort"] = p.getJointState(self.robot, 3)[3]
        self.observation["wheel_l_effort"] = p.getJointState(self.robot, 2)[3]
        self.observation["wheel_r_effort"] = p.getJointState(self.robot, 5)[3]
        time.sleep(1.0 / self.control_rate_hz)


if __name__ == "__main__":
    interface = HardwareInterfaceSimulator()
    while True:
        interface.update(action=np.zeros(1, dtype=action_dtype))
