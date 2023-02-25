#ifndef DATA_TYPES_H
#define DATA_TYPES_H
#include <data_types.h>
#endif

class Controller
{
public:
    Controller(const RobotConfig &robot_config);
    void update(const RobotObservation &robot_obs, const RobotCommand &robot_cmd, RobotAction &robot_act);
    void update_odometry(const RobotObservation &robot_obs, const RobotCommand &robot_cmd);
    void reset_odometry();
    double pd_control(double q, double qd, double q_des, double qd_des, double kp, double kd);
    std::tuple<double, double> wheel_to_odom(double q_r, double q_l);
    std::tuple<double, double> odom_to_wheel(double x, double yaw);
    std::tuple<double, double> z_to_legs(double z);
    double legs_to_z(double q_r, double q_l);
private:
    RobotConfig robot_config;
    Odometry odometry_state;
    Odometry odometry_command;
};