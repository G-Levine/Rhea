#include <map>
#include <string>


struct ActuatorObservation
{
    double pos;
    double vel;
    double torque;
};

struct ActuatorAction
{
    double pos;
    double vel;
    double torque;
    double kp;
    double kd;
};

struct MsgFormatConfig
{
    double pos_min;
    double pos_max;
    double vel_min;
    double vel_max;
    double torque_min;
    double torque_max;
    double kp_min;
    double kp_max;
    double kd_min;
    double kd_max;
    int axis_dir; // 1 or -1
};

struct ActuatorConfig
{
    std::string name;
    double pos_default;
    int id;
    int bus;
    MsgFormatConfig msg_format;
};

struct IMUObservation
{
    double roll;
    double pitch;
    double roll_vel;
    double pitch_vel;
};

struct RobotObservation
{
    std::map<std::string, ActuatorObservation> actuator_obs;
    IMUObservation imu_obs;
};

struct Odometry
{
    double x;
    double x_vel;
    double yaw;
    double yaw_vel;
};

struct PDGains
{
    double kp;
    double kd;
};

struct RobotAction
{
    std::map<std::string, ActuatorAction> actuator_actions;
};

struct RobotConfig
{
    double control_freq;
    double wheel_radius; // radius of each wheel in meters
    double wheel_base; // half of the lateral distance between the left and right wheels in meters
    double link_length; // length of each leg link in meters
    double contact_threshold; // threshold for detecting contact in Newtons
    std::map<std::string, ActuatorConfig> actuator_configs;
    PDGains pitch_gains;
    PDGains x_gains;
    PDGains yaw_gains;
    PDGains leg_gains;
};

struct RobotCommand
{
    double x_vel;
    double yaw_vel;
    double z_pos;
};

struct UserInput
{
    double x_input;
    double yaw_input;
    bool enable_toggle;
    bool stand_toggle;
};

enum OpMode
{
    kIdle,
    kStandUp,
    kRun,
    kSitDown,
};