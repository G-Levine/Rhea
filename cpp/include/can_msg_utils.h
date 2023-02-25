#include <cstdint>

#ifndef DATA_TYPES_H
#define DATA_TYPES_H
#include <data_types.h>
#endif

/*
This file contains functions for encoding and decoding CAN messages.
*/

/*
Hardcoded utility messages.
*/
const unsigned char enable_msg[8] = {0xFF, 0xFF, 0xFF, 0xFF,
                                  0xFF, 0xFF, 0xFF, 0xFC};
const unsigned char disable_msg[8] = {0xFF, 0xFF, 0xFF, 0xFF,
                                   0xFF, 0xFF, 0xFF, 0xFD};
const unsigned char zero_msg[8] = {0xFF, 0xFF, 0xFF, 0xFF,
                                0xFF, 0xFF, 0xFF, 0xFE};

/*
This function encodes an ActuatorAction struct to the Mini Cheetah CAN command format and writes it to a byte array.
*/
void encode_actuator_action(const ActuatorAction &actuator_act, const ActuatorConfig &actuator_config, uint8_t *CAN_msg);

/*
This function decodes a byte array in the Mini Cheetah CAN response format to an ActuatorObservation struct and writes it to the robot observation.
*/
void decode_actuator_observation(const uint8_t *CAN_msg, const RobotConfig &robot_config, RobotObservation &robot_obs);

int double_to_uint(double x, double x_min, double x_max, int bits);
float uint_to_double(int x_int, double x_min, double x_max, int bits);
