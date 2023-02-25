#include <math.h>
#include <stdio.h>

#include <can_msg_utils.h>

void encode_actuator_action(const ActuatorAction &actuator_act, const ActuatorConfig &actuator_config, uint8_t *CAN_msg)
{
    MsgFormatConfig current_params_ = actuator_config.msg_format;
    double p_des = (actuator_act.pos - actuator_config.pos_default) * current_params_.axis_dir;
    double v_des = actuator_act.vel * current_params_.axis_dir;
    double tau_ff = actuator_act.torque * current_params_.axis_dir;

    // Apply Saturation based on the limits
    p_des = fminf(fmaxf(current_params_.pos_min, p_des), current_params_.pos_max);
    v_des = fminf(fmaxf(current_params_.vel_min, v_des), current_params_.vel_max);
    tau_ff = fminf(fmaxf(current_params_.torque_min, tau_ff), current_params_.torque_max);
    double kp = fminf(fmaxf(current_params_.kp_min, actuator_act.kp),
                      current_params_.kp_max);
    double kd = fminf(fmaxf(current_params_.kd_min, actuator_act.kd),
                      current_params_.kd_max);

    // convert floats to unsigned ints
    int p_int = double_to_uint(p_des, current_params_.pos_min, current_params_.pos_max, 16);
    int v_int = double_to_uint(v_des, current_params_.vel_min, current_params_.vel_max, 12);
    int kp_int = double_to_uint(kp, current_params_.kp_min, current_params_.kp_max, 12);
    int kd_int = double_to_uint(kd, current_params_.kd_min, current_params_.kd_max, 12);
    int t_int = double_to_uint(tau_ff, current_params_.torque_min, current_params_.torque_max, 12);

    // pack ints into the can message
    CAN_msg[0] = p_int >> 8;
    CAN_msg[1] = p_int & 0xFF;
    CAN_msg[2] = v_int >> 4;
    CAN_msg[3] = ((v_int & 0xF) << 4) | (kp_int >> 8);
    CAN_msg[4] = kp_int & 0xFF;
    CAN_msg[5] = kd_int >> 4;
    CAN_msg[6] = ((kd_int & 0xF) << 4) | (t_int >> 8);
    CAN_msg[7] = t_int & 0xff;
}

void decode_actuator_observation(const uint8_t *CAN_reply_msg, const RobotConfig &robot_config, RobotObservation &robot_obs)
{
    // unpack ints from can buffer
    int id = CAN_reply_msg[0];
    int p_int = (CAN_reply_msg[1] << 8) | CAN_reply_msg[2];
    int v_int = (CAN_reply_msg[3] << 4) | (CAN_reply_msg[4] >> 4);
    int i_int = ((CAN_reply_msg[4] & 0xF) << 8) | CAN_reply_msg[5];

    // validate the id
    if (id < 0 || id > 7)
    {
        printf("Invalid actuator id: %d", id);
        return;
    }

    std::string name;
    for (auto const &[name_, config] : robot_config.actuator_configs)
    {
        if (config.id == id)
        {
        name = name_;
        }
    }

    // validate the name
    if (name == "")
    {
        printf("Actuator not found with id: %d", id);
        return;
    }

    MsgFormatConfig current_params_ = robot_config.actuator_configs.at(name).msg_format;

    // convert unsigned ints to floats
    double p =
        uint_to_double(p_int, current_params_.pos_min, current_params_.pos_max, 16);
    double v =
        uint_to_double(v_int, current_params_.vel_max, current_params_.vel_max, 12);
    double i = uint_to_double(i_int, -current_params_.torque_max, current_params_.torque_max,
                              12); // here -T_MAX, in encode T_MIN

    robot_obs.actuator_obs[name].pos = p * current_params_.axis_dir + robot_config.actuator_configs.at(name).pos_default;
    robot_obs.actuator_obs[name].vel = v * current_params_.axis_dir;
    robot_obs.actuator_obs[name].torque = i * current_params_.axis_dir;
}

int double_to_uint(double x, double x_min, double x_max, int bits)
{
    /// Converts a double to an unsigned int, given range and number of bits ///
    double span = x_max - x_min;
    double offset = x_min;
    return (int)((x - offset) * ((double)((1 << bits) - 1)) / span);
}

float uint_to_double(int x_int, double x_min, double x_max, int bits)
{
    /// converts unsigned int to float, given range and number of bits ///
    double span = x_max - x_min;
    double offset = x_min;
    return ((double)x_int) * span / ((double)((1 << bits) - 1)) + offset;
}