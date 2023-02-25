import numpy as np
import asyncio
import time
import yaml
import atexit

import moteus_pi3hat
import moteus

from data_types import *

import canmotorlib

SLEEP_TIME = 0.1

class HardwareInterface:

    async def start_all(self,):
        for command in self.commands:
            command.data = canmotorlib.ZERO_MOTOR
            command.reply_required = False
        await self.transport.cycle(self.commands, request_attitude=True)
        time.sleep(SLEEP_TIME)
        for command in self.commands:
            command.data = canmotorlib.ENABLE_MOTOR
            command.reply_required = True
        await self.transport.cycle(self.commands, request_attitude=True)
        time.sleep(SLEEP_TIME)


    async def stop_all(self):
        # Zero the commands first
        for command in self.commands:
            command.data = canmotorlib.ZERO_MOTOR
            command.reply_required = False
        await self.transport.cycle(self.commands, request_attitude=True)
        time.sleep(SLEEP_TIME)
        # Disable the motors
        for command in self.commands:
            command.data = canmotorlib.DISABLE_MOTOR
            command.reply_required = False
        await self.transport.cycle(self.commands, request_attitude=True)
        time.sleep(SLEEP_TIME)


    def id_to_idx(self, id):
        return id - 1


    def idx_to_id(self, idx):
        return idx + 1


    def __init__(self):
        self.observation = np.zeros(1, dtype=observation_dtype)

        # Load the configuration file
        with open('config.yaml') as f:
            self.config = yaml.load(f, Loader=yaml.FullLoader)

        # Create the actuator and command objects
        self.actuators = []
        self.actuator_names = []
        self.actuator_zero_offsets = []
        self.commands = []
        for actuator_name, actuator in self.config['actuators'].items():
            self.actuator_names.append(actuator_name)
            self.actuators.append(canmotorlib.CanMotorController(
                motor_type=actuator['type']))
            self.actuator_zero_offsets.append(actuator['zero_offset_deg'] * np.pi / 180)
            command = moteus.Command()
            command.raw = True
            command.arbitration_id = actuator['id']
            command.bus = actuator['bus']
            command.reply_required = False
            self.commands.append(command)

        # Create the state and command arrays
        self.state_positions = np.zeros(len(self.actuators))
        self.state_velocities = np.zeros(len(self.actuators))
        self.state_torques = np.zeros(len(self.actuators))

        self.command_positions = np.zeros(len(self.actuators))
        self.command_velocities = np.zeros(len(self.actuators))
        self.command_torques = np.zeros(len(self.actuators))
        self.command_kps = np.zeros(len(self.actuators))
        self.command_kds = np.zeros(len(self.actuators))

        # Configure the CAN bus
        single_can_config = moteus_pi3hat.CanConfiguration()
        single_can_config.fdcan_frame = False
        single_can_config.bitrate_switch = False
        can_config = {i: single_can_config for i in range(5)}

        # Configure the mounting angle
        # Extrinsic Euler angles in order of roll, pitch, yaw
        mounting_config = {
            'roll': 0,
            'pitch': 0,
            'yaw': 0,
            # 'roll': 180,
            # 'pitch': 0,
            # 'yaw': 90,
        }

        self.transport = moteus_pi3hat.Pi3HatRouter(
            can=can_config, mounting_deg=mounting_config, attitude_rate_hz=1000)

        # Register the exit function
        atexit.register(self.stop_all)

    async def update(self, action):
        # Update the commands
        self.command_positions[0] = action["desired_leg_1_r_pos"] - self.actuator_zero_offsets[0]
        self.command_positions[2] = action["desired_leg_1_l_pos"] - self.actuator_zero_offsets[2]
        self.command_kps[0] = action["desired_leg_1_r_kp"]
        self.command_kps[2] = action["desired_leg_1_l_kp"]
        self.command_kds[0] = action["desired_leg_1_r_kd"]
        self.command_kds[2] = action["desired_leg_1_l_kd"]
        self.command_torques[1] = action["desired_wheel_r_effort"]
        self.command_torques[3] = action["desired_wheel_l_effort"]

        for i, command in enumerate(self.commands):
            command.data = self.actuators[i].encode_command(
                self.command_positions[i],
                self.command_velocities[i],
                self.command_kps[i],
                self.command_kds[i],
                self.command_torques[i],
            )
            command.reply_required = True

        result = await self.transport.cycle(self.commands, request_attitude=True)

        # Update the state
        for x in result:
            # Check if the result is the IMU
            if isinstance(x, moteus_pi3hat.CanAttitudeWrapper):
                imu_result = x
            # Otherwise, it is a motor
            elif x.data:
                try:
                    # Get the motor ID
                    motor_id, _, _, _ = self.actuators[0].decode_response(x.data)
                    idx = self.id_to_idx(motor_id)
                    # Decode the response
                    motor_id, position, velocity, torque = self.actuators[idx].decode_response(
                        x.data)
                    self.state_positions[idx] = position + \
                        self.actuator_zero_offsets[idx]
                    self.state_velocities[idx] = velocity
                    self.state_torques[idx] = torque
                except ValueError:
                    print("failed to decode response")
                    pass

        if imu_result:
            att = imu_result.attitude
            rate_dps = imu_result.rate_dps
            accel_mps2 = imu_result.accel_mps2
            euler_rad = imu_result.euler_rad

        roll = euler_rad.pitch
        pitch = euler_rad.roll + np.pi / 2
        yaw = euler_rad.yaw
        roll_rate = rate_dps.z * np.pi / 180
        pitch_rate = rate_dps.x * np.pi / 180
        yaw_rate = rate_dps.y * np.pi / 180

        # # Print the Euler angles
        # print(f"Roll: {roll * 180 / np.pi:.2f} deg")
        # print(f"Pitch: {pitch * 180 / np.pi:.2f} deg")
        # print(f"Yaw: {yaw * 180 / np.pi:.2f} deg")
        # # Print the angular rates
        # print(f"Roll rate: {roll_rate * 180 / np.pi:.2f} deg/s")
        # print(f"Pitch rate: {pitch_rate * 180 / np.pi:.2f} deg/s")
        # print(f"Yaw rate: {yaw_rate * 180 / np.pi:.2f} deg/s")
        # print()

        self.observation["roll"] = roll
        self.observation["pitch"] = pitch
        self.observation["roll_vel"] = roll_rate
        self.observation["pitch_vel"] = pitch_rate
        self.observation["leg_1_r_pos"] = self.state_positions[0]
        self.observation["wheel_r_pos"] = self.state_positions[1]
        self.observation["leg_1_l_pos"] = self.state_positions[2]
        self.observation["wheel_l_pos"] = self.state_positions[3]
        self.observation["leg_1_r_vel"] = self.state_velocities[0]
        self.observation["wheel_r_vel"] = self.state_velocities[1]
        self.observation["leg_1_l_vel"] = self.state_velocities[2]
        self.observation["wheel_l_vel"] = self.state_velocities[3]
        self.observation["leg_1_r_effort"] = self.state_torques[0]
        self.observation["wheel_r_effort"] = self.state_torques[1]
        self.observation["leg_1_l_effort"] = self.state_torques[2]
        self.observation["wheel_l_effort"] = self.state_torques[3]

