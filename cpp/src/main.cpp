/*
This file defines the main control loop for the Rhea robot.
*/
#ifndef DATA_TYPES_H
#define DATA_TYPES_H
#include <data_types.h>
#endif

#include <controller.h>
#include <hardware_interface.h>

#include <boost/asio.hpp>
#include <boost/date_time/posix_time/posix_time.hpp>

#include <yaml-cpp/yaml.h>

int main(int argc, char **argv)
{
    // Initialize the robot configuration
    RobotConfig robot_config;

    // Load the robot configuration from a yaml file
    YAML::Node config = YAML::LoadFile("config.yaml");
    robot_config.control_freq = config["control_freq"].as<int>();
    robot_config.wheel_radius = config["wheel_radius"].as<double>();
    robot_config.wheel_base = config["wheel_base"].as<double>();
    robot_config.contact_threshold = config["contact_threshold"].as<double>();
    robot_config.pitch_gains.kp = config["pitch_gains"]["kp"].as<double>();
    robot_config.pitch_gains.kd = config["pitch_gains"]["kd"].as<double>();
    robot_config.x_gains.kp = config["x_gains"]["kp"].as<double>();
    robot_config.x_gains.kd = config["x_gains"]["kd"].as<double>();
    robot_config.yaw_gains.kp = config["yaw_gains"]["kp"].as<double>();
    robot_config.yaw_gains.kd = config["yaw_gains"]["kd"].as<double>();
    robot_config.leg_gains.kp = config["leg_gains"]["kp"].as<double>();
    robot_config.leg_gains.kd = config["leg_gains"]["kd"].as<double>();
    for (YAML::const_iterator it = config["actuator_configs"].begin(); it != config["actuator_configs"].end(); ++it)
    {
        std::string actuator_name = it->first.as<std::string>();
        robot_config.actuator_configs[actuator_name].bus = config["actuator_configs"][actuator_name]["bus"].as<int>();
        robot_config.actuator_configs[actuator_name].msg_format = config["actuator_configs"][actuator_name]["msg_format"].as<MsgFormatConfig>();
        robot_config.actuator_configs[actuator_name].id = config["actuator_configs"][actuator_name]["id"].as<int>();
        robot_config.actuator_configs[actuator_name].pos_default = config["actuator_configs"][actuator_name]["pos_default"].as<double>();
    }

    // Initialize the robot observation
    RobotObservation robot_obs;
    // Initialize the robot command
    RobotCommand robot_cmd;
    // Initialize the robot action
    RobotAction robot_act;

    // Initialize the controller
    Controller controller(robot_config);
    // Initialize the hardware interface
    HardwareInterface hardware_interface(robot_config);
    // Enable the hardware interface
    hardware_interface.enable();
    // Zero the hardware interface
    hardware_interface.zero();

    boost::asio::io_service io;
    int loop_time = 1000 / robot_config.control_freq; // Loop time in milliseconds

    // Run the main control loop
    while (true)
    {
        boost::asio::deadline_timer t(io, boost::posix_time::millisec(loop_time));
        // Nonblocking read of the input command

        // Update the controller
        controller.update(robot_obs, robot_cmd, robot_act);
        // Update the hardware interface
        hardware_interface.update(robot_act, robot_obs);
        // Nonblocking write of the telemetry data
        
        t.wait();
    }
    return 0;
}