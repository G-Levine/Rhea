import socket
import struct
import time
import math
from bitstring import BitArray

# Initial Code Taken From: https://elinux.org/Python_Can


# CAN frame packing/unpacking (see `struct can_frame` in <linux/can.h>)
# 8 bytes of data is sent to the motor
can_frame_fmt_send = "=IB3x8s"
# 6 bytes are received from the motor
can_frame_fmt_recv = "=IB3x6s"
# Total CAN Frame size is 14 Bytes: 8 Bytes overhead + 6 Bytes data
recvBytes = 14


ENABLE_MOTOR = b'\xFF\xFF\xFF\xFF\xFF\xFF\xFF\xFC'
DISABLE_MOTOR = b'\xFF\xFF\xFF\xFF\xFF\xFF\xFF\xFD'
ZERO_MOTOR = b'\xFF\xFF\xFF\xFF\xFF\xFF\xFF\xFE'

# List of Motors Supported by this Driver
legitimate_motors = [
                    "GIM8115_9",
                    "GIM6010_6",
                    ]

# Constants for conversion

GIM8115_9_PARAMS = {
                "P_MIN" : -95.5,
                "P_MAX" : 95.5,
                "V_MIN" : -30.0,
                "V_MAX" : 30.0,
                "KP_MIN" : 0.0,
                "KP_MAX" : 500,
                "KD_MIN" : 0.0,
                "KD_MAX" : 5.0,
                "T_MIN" : -18.0,
                "T_MAX" : 18.0,
                "AXIS_DIRECTION" : -1
                }


GIM6010_6_PARAMS = {
                "P_MIN" : -95.5,
                "P_MAX" : 95.5,
                "V_MIN" : -30.0,
                "V_MAX" : 30.0,
                "KP_MIN" : 0.0,
                "KP_MAX" : 500,
                "KD_MIN" : 0.0,
                "KD_MAX" : 5.0,
                "T_MIN" : -18.0,
                "T_MAX" : 18.0,
                "AXIS_DIRECTION" : -1
                }


maxRawPosition = 2**16 - 1                      # 16-Bits for Raw Position Values
maxRawVelocity = 2**12 - 1                      # 12-Bits for Raw Velocity Values
maxRawTorque = 2**12 - 1                        # 12-Bits for Raw Torque Values
maxRawKp = 2**12 - 1                            # 12-Bits for Raw Kp Values
maxRawKd = 2**12 - 1                            # 12-Bits for Raw Kd Values
maxRawCurrent = 2**12 - 1                       # 12-Bits for Raw Current Values
dt_sleep = 0.0001                               # Time before motor sends a reply


def float_to_uint(x, x_min, x_max, numBits):
    span = x_max - x_min
    offset = x_min
    # Attempt to speedup by using pre-computation. Not used currently.
    if numBits == 16:
        bitRange = maxRawPosition
    elif numBits == 12:
        bitRange = maxRawVelocity
    else:
        bitRange = 2**numBits - 1
    return int(((x - offset) * (bitRange)) / span)


def uint_to_float(x_int, x_min, x_max, numBits):
    span = x_max - x_min
    offset = x_min
    if numBits == 16:
        bitRange = maxRawPosition
    elif numBits == 12:
        bitRange = maxRawVelocity
    else:
        bitRange = 2**numBits - 1
    return ((x_int * span) / (bitRange)) + offset


class CanMotorController:
    """
    Class for creating a Mini-Cheetah Motor Controller over CAN. Uses SocketCAN driver for
    communication.
    """

    def __init__(self, motor_type="GIM8115_9"):
        """
        Instantiate the class with socket name, motor ID, and socket timeout.
        Sets up the socket communication for rest of the functions.
        """
        self.motorParams = GIM8115_9_PARAMS  # default choice
        print("Using Motor Type: {}".format(motor_type))
        assert motor_type in legitimate_motors, "Motor Type not in list of accepted motors."
        if motor_type == "GIM6010_6_PARAMS":
            self.motorParams = GIM6010_6_PARAMS

        # Initialize the command BitArrays for performance optimization
        self._p_des_BitArray = BitArray(
            uint=float_to_uint(0, self.motorParams["P_MIN"], self.motorParams["P_MAX"], 16), length=16
        )
        self._v_des_BitArray = BitArray(
            uint=float_to_uint(0, self.motorParams["V_MIN"], self.motorParams["V_MAX"], 12), length=12
        )
        self._kp_BitArray = BitArray(uint=0, length=12)
        self._kd_BitArray = BitArray(uint=0, length=12)
        self._tau_BitArray = BitArray(uint=0, length=12)
        self._cmd_bytes = BitArray(uint=0, length=64)
        self._recv_bytes = BitArray(uint=0, length=48)

    def decode_response(self, data_frame):
        """
        Function to decode the motor status reply message into its constituent raw values.

        /// CAN Reply Packet Structure ///
        /// 16 bit position, between -4*pi and 4*pi
        /// 12 bit velocity, between -30 and + 30 rad/s
        /// 12 bit current, between -40 and 40;
        /// CAN Packet is 5 8-bit words
        /// Formatted as follows.  For each quantity, bit 0 is LSB
        /// 0: [position[15-8]]
        /// 1: [position[7-0]]
        /// 2: [velocity[11-4]]
        /// 3: [velocity[3-0], current[11-8]]
        /// 4: [current[7-0]]

        returns: the following raw values as (u)int: position, velocity, current
        """

        # Convert the message from motor to a bit string as this is easier to deal with than hex
        # while seperating individual values.
        self._recv_bytes.bytes = data_frame
        dataBitArray = self._recv_bytes.bin

        # Separate motor status values from the bit string.
        # Motor ID not considered necessary at the moment.
        motor_id = dataBitArray[:8]
        positionBitArray = dataBitArray[8:24]
        velocityBitArray = dataBitArray[24:36]
        currentBitArray = dataBitArray[36:48]

        motor_id = int(motor_id, 2)
        positionRawValue = int(positionBitArray, 2)
        velocityRawValue = int(velocityBitArray, 2)
        currentRawValue = int(currentBitArray, 2)

        # TODO: Is it necessary/better to return motor_id?
        # return motor_id, positionRawValue, velocityRawValue, currentRawValue
        return self.convert_raw_to_physical_rad(motor_id, positionRawValue, velocityRawValue, currentRawValue)

    def convert_raw_to_physical_rad(self, motor_id, positionRawValue, velocityRawValue, currentRawValue):
        """
        Function to convert the raw values from the motor to physical values:

        /// CAN Reply Packet Structure ///
        /// 16 bit position, between -4*pi and 4*pi
        /// 12 bit velocity, between -30 and + 30 rad/s
        /// 12 bit current, between -40 and 40;
        /// CAN Packet is 5 8-bit words
        /// Formatted as follows.  For each quantity, bit 0 is LSB
        /// 0: [position[15-8]]
        /// 1: [position[7-0]]
        /// 2: [velocity[11-4]]
        /// 3: [velocity[3-0], current[11-8]]
        /// 4: [current[7-0]]

        returns: position (radians), velocity (rad/s), current (amps)
        """

        physicalPositionRad = uint_to_float(positionRawValue, self.motorParams["P_MIN"], self.motorParams["P_MAX"], 16)
        physicalVelocityRad = uint_to_float(velocityRawValue, self.motorParams["V_MIN"], self.motorParams["V_MAX"], 12)
        physicalCurrent = uint_to_float(currentRawValue, self.motorParams["T_MIN"], self.motorParams["T_MAX"], 12)

        # Correct Axis Direction
        physicalPositionRad = physicalPositionRad * self.motorParams["AXIS_DIRECTION"]
        physicalVelocityRad = physicalVelocityRad * self.motorParams["AXIS_DIRECTION"]
        physicalCurrent = physicalCurrent * self.motorParams["AXIS_DIRECTION"]

        return motor_id, physicalPositionRad, physicalVelocityRad, physicalCurrent

    def convert_physical_rad_to_raw(self, p_des_rad, v_des_rad, kp, kd, tau_ff):

        # Correct the Axis Direction
        p_des_rad = p_des_rad * self.motorParams["AXIS_DIRECTION"]
        v_des_rad = v_des_rad * self.motorParams["AXIS_DIRECTION"]
        tau_ff = tau_ff * self.motorParams["AXIS_DIRECTION"]

        rawPosition = float_to_uint(p_des_rad, self.motorParams["P_MIN"], self.motorParams["P_MAX"], 16)
        rawVelocity = float_to_uint(v_des_rad, self.motorParams["V_MIN"], self.motorParams["V_MAX"], 12)
        rawTorque = float_to_uint(tau_ff, self.motorParams["T_MIN"], self.motorParams["T_MAX"], 12)

        rawKp = (maxRawKp * kp) / self.motorParams["KP_MAX"]

        rawKd = (maxRawKd * kd) / self.motorParams["KD_MAX"]

        return int(rawPosition), int(rawVelocity), int(rawKp), int(rawKd), int(rawTorque)

    def get_raw_command(self, p_des, v_des, kp, kd, tau_ff):
        """
        Package and send raw (uint) values of correct length to the motor.

        _send_raw_command(desired position, desired velocity, position gain, velocity gain,
                        feed-forward torque)

        Sends data over CAN, reads response, and returns the motor status data (in bytes).
        """
        self._p_des_BitArray.uint = p_des
        self._v_des_BitArray.uint = v_des
        self._kp_BitArray.uint = kp
        self._kd_BitArray.uint = kd
        self._tau_BitArray.uint = tau_ff
        cmd_BitArray = (
            self._p_des_BitArray.bin
            + self._v_des_BitArray.bin
            + self._kp_BitArray.bin
            + self._kd_BitArray.bin
            + self._tau_BitArray.bin
        )

        self._cmd_bytes.bin = cmd_BitArray

        return self._cmd_bytes.tobytes()

    def encode_command(self, p_des_rad, v_des_rad, kp, kd, tau_ff):
        """
        Function to send data to motor in physical units:
        send_rad_command(position (rad), velocity (rad/s), kp, kd, Feedforward Torque (Nm))
        Sends data over CAN, reads response, and prints the current status in rad, rad/s, amps.
        If any input is outside limits, it is clipped. Only if torque is outside limits, a log
        message is shown.
        """
        # Check for Torque Limits
        if tau_ff < self.motorParams["T_MIN"]:
            print("Torque Commanded lower than the limit. Clipping Torque...")
            print("Commanded Torque: {}".format(tau_ff))
            print("Torque Limit: {}".format(self.motorParams["T_MIN"]))
            tau_ff = self.motorParams["T_MIN"]
        elif tau_ff > self.motorParams["T_MAX"]:
            print("Torque Commanded higher than the limit. Clipping Torque...")
            print("Commanded Torque: {}".format(tau_ff))
            print("Torque Limit: {}".format(self.motorParams["T_MAX"]))
            tau_ff = self.motorParams["T_MAX"]

        # Clip Position if outside Limits
        p_des_rad = min(max(self.motorParams["P_MIN"], p_des_rad), self.motorParams["P_MAX"])
        # Clip Velocity if outside Limits
        v_des_rad = min(max(self.motorParams["V_MIN"], v_des_rad), self.motorParams["V_MAX"])
        # Clip Kp if outside Limits
        kp = min(max(self.motorParams["KP_MIN"], kp), self.motorParams["KP_MAX"])
        # Clip Kd if outside Limits
        kd = min(max(self.motorParams["KD_MIN"], kd), self.motorParams["KD_MAX"])

        rawPos, rawVel, rawKp, rawKd, rawTauff = self.convert_physical_rad_to_raw(p_des_rad, v_des_rad, kp, kd, tau_ff)

        command_bytes = self.get_raw_command(rawPos, rawVel, rawKp, rawKd, rawTauff)
        return command_bytes

    def change_motor_constants(
        self,
        P_MIN_NEW,
        P_MAX_NEW,
        V_MIN_NEW,
        V_MAX_NEW,
        KP_MIN_NEW,
        KP_MAX_NEW,
        KD_MIN_NEW,
        KD_MAX_NEW,
        T_MIN_NEW,
        T_MAX_NEW,
    ):
        """
        Function to change the global motor constants. Default values are for AK80-6 motor from
        CubeMars. For a different motor, the min/max values can be changed here for correct
        conversion.
        change_motor_params(P_MIN_NEW (radians), P_MAX_NEW (radians), V_MIN_NEW (rad/s),
                            V_MAX_NEW (rad/s), KP_MIN_NEW, KP_MAX_NEW, KD_MIN_NEW, KD_MAX_NEW,
                            T_MIN_NEW (Nm), T_MAX_NEW (Nm))
        """
        self.motorParams["P_MIN"] = P_MIN_NEW
        self.motorParams["P_MAX"] = P_MAX_NEW
        self.motorParams["V_MIN"] = V_MIN_NEW
        self.motorParams["V_MAX"] = V_MAX_NEW
        self.motorParams["KP_MIN"] = KP_MIN_NEW
        self.motorParams["KP_MAX"] = KP_MAX_NEW
        self.motorParams["KD_MIN"] = KD_MIN_NEW
        self.motorParams["KD_MAX"] = KD_MAX_NEW
        self.motorParams["T_MIN"] = T_MIN_NEW
        self.motorParams["T_MAX"] = T_MAX_NEW