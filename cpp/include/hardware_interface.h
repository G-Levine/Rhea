#ifndef DATA_TYPES_H
#define DATA_TYPES_H
#include <data_types.h>
#endif

#include <pi3hat.h>

class HardwareInterface
{
public:
    HardwareInterface(const RobotConfig &robot_config);
    void update(const RobotAction &robot_act, RobotObservation &robot_obs);
    void enable();
    void disable();
    void zero();
private:
    RobotConfig robot_config;
    mjbots::pi3hat::Pi3Hat* pi3hat;
    mjbots::pi3hat::Pi3Hat::Input pi3hat_input;
    mjbots::pi3hat::Pi3Hat::Output pi3hat_output;
};

mjbots::pi3hat::Euler quat_to_euler(mjbots::pi3hat::Quaternion q);