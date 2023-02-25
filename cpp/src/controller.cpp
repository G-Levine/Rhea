#include "controller.h"
#include <math.h>

Controller::Controller(const RobotConfig &robot_config)
{
    this->robot_config = robot_config;
    reset_odometry();
}

void Controller::update(const RobotObservation &robot_obs, const RobotCommand &robot_cmd, RobotAction &robot_act)
{
    // State estimation
    // Odometry
    update_odometry(robot_obs, robot_cmd);

    // Contact state
    

    // Control
    // Leg controller
    double action_leg_1_r_pos, action_leg_1_l_pos;
    std::tie(action_leg_1_r_pos, action_leg_1_l_pos) = z_to_legs(robot_cmd.z_pos);

    robot_act.actuator_actions.at("leg_1_r").pos = action_leg_1_r_pos;
    robot_act.actuator_actions.at("leg_1_l").pos = action_leg_1_l_pos;

    // Yaw controller
    double yaw_kp = robot_config.yaw_gains.kp;
    double yaw_kd = robot_config.yaw_gains.kd;
    double yaw_torque = pd_control(odometry_state.yaw, odometry_state.yaw_vel, odometry_command.yaw, robot_cmd.yaw_vel, robot_config.yaw_gains.kp, robot_config.yaw_gains.kd);

    // Balance controller
    double pitch_kp = robot_config.pitch_gains.kp;
    double pitch_kd = robot_config.pitch_gains.kd;
    double pitch_torque = pd_control(robot_obs.imu_obs.pitch, robot_obs.imu_obs.pitch_vel, 0, 0, robot_config.pitch_gains.kp, robot_config.pitch_gains.kd);

    double x_kp = robot_config.x_gains.kp;
    double x_kd = robot_config.x_gains.kd;
    double x_torque = pd_control(odometry_state.x, odometry_state.x_vel, odometry_command.x, robot_cmd.x_vel, robot_config.x_gains.kp, robot_config.x_gains.kd);

    double balance_torque = pitch_torque + x_torque;

    double action_wheel_r_torque = yaw_torque + balance_torque;
    double action_wheel_l_torque = yaw_torque - balance_torque;

    robot_act.actuator_actions.at("wheel_r").torque = action_wheel_r_torque;
    robot_act.actuator_actions.at("wheel_l").torque = action_wheel_l_torque;
}

void Controller::update_odometry(const RobotObservation &robot_obs, const RobotCommand &robot_cmd)
{
    // Get wheel velocities
    double wheel_r_vel = robot_obs.actuator_obs.at("wheel_r").vel;
    double wheel_l_vel = robot_obs.actuator_obs.at("wheel_l").vel;

    // Convert wheel velocities to odometry
    double x_vel, yaw_vel;
    std::tie(x_vel, yaw_vel) = wheel_to_odom(wheel_r_vel, wheel_l_vel);

    // Update odometry state
    odometry_state.x_vel = x_vel;
    odometry_state.yaw_vel = yaw_vel;
    odometry_state.x += odometry_state.x_vel / robot_config.control_freq;
    odometry_state.yaw += odometry_state.yaw_vel / robot_config.control_freq;

    // Update odometry command
    odometry_command.x_vel = robot_cmd.x_vel;
    odometry_command.yaw_vel = robot_cmd.yaw_vel;
    odometry_command.x += odometry_command.x_vel / robot_config.control_freq;
    odometry_command.yaw += odometry_command.yaw_vel / robot_config.control_freq;
}

void Controller::reset_odometry()
{
    odometry_state.x = 0.0;
    odometry_state.yaw = 0.0;
    odometry_state.x_vel = 0.0;
    odometry_state.yaw_vel = 0.0;

    odometry_command.x = 0.0;
    odometry_command.yaw = 0.0;
    odometry_command.x_vel = 0.0;
    odometry_command.yaw_vel = 0.0;
}

double Controller::pd_control(double q, double qd, double q_des, double qd_des, double kp, double kd)
{
    return kp * (q_des - q) + kd * (qd_des - qd);
}

std::tuple<double, double> Controller::wheel_to_odom(double q_r, double q_l)
{
    double x = (q_l - q_r) * M_PI * robot_config.wheel_radius / 2.0;
    double yaw = (q_l + q_r) * M_PI * robot_config.wheel_radius / (robot_config.wheel_base * 2.0);
    return std::make_pair(x, yaw);
}

std::tuple<double, double> Controller::odom_to_wheel(double x, double yaw)
{
    double q_r = (yaw * robot_config.wheel_base - x) / (M_PI * robot_config.wheel_radius);
    double q_l = (yaw * robot_config.wheel_base + x) / (M_PI * robot_config.wheel_radius);
    return std::make_pair(q_r, q_l);
}

std::tuple<double, double> Controller::z_to_legs(double z)
{
    double q_r = -acos(z / (robot_config.link_length * 2.0));
    double q_l = acos(z / (robot_config.link_length * 2.0));
    return std::make_pair(q_r, q_l);
}

double Controller::legs_to_z(double q_r, double q_l)
{
    double cos_q_avg = (-cos(q_r) + cos(q_l)) / 2.0;
    double z = 2.0 * robot_config.link_length * cos_q_avg;
    return z;
}