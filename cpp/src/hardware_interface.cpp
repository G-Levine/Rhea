#include <hardware_interface.h>
#include <can_msg_utils.h>
#include <math.h>

HardwareInterface::HardwareInterface(const RobotConfig &robot_config)
{
  this->robot_config = robot_config;

  mjbots::pi3hat::Pi3Hat::CanConfiguration can_configuration;
  can_configuration.fdcan_frame = false;
  can_configuration.bitrate_switch = false;

  mjbots::pi3hat::Pi3Hat::Configuration configuration;
  // loop through and configure the 5 CAN buses
  for (int i = 0; i < 5; i++)
  {
    configuration.can[i] = can_configuration;
  }

  configuration.attitude_rate_hz = 1000;
  configuration.mounting_deg = mjbots::pi3hat::Euler(0.0, 0.0, 0.0);

  // Initialize the Pi3Hat
  pi3hat = new mjbots::pi3hat::Pi3Hat(configuration);

  // Initialize the Pi3Hat input and output
  pi3hat_input = mjbots::pi3hat::Pi3Hat::Input();
  pi3hat_output = mjbots::pi3hat::Pi3Hat::Output();
  pi3hat_input.request_attitude = true;
}

void HardwareInterface::update(const RobotAction &robot_act, RobotObservation &robot_obs)
{
  // Process each action
  for (auto const &[name, action] : robot_act.actuator_actions)
  {
    ActuatorConfig config = robot_config.actuator_configs.at(name);
    int can_index = config.id - 1;
    pi3hat_input.tx_can[can_index].id = config.id;
    pi3hat_input.tx_can[can_index].expect_reply = true;
    uint8_t *data = pi3hat_input.tx_can[can_index].data;
    // Encode the action
    encode_actuator_action(action, config, data);
  }
  // Update the Pi3Hat
  const auto result = pi3hat->Cycle(pi3hat_input);
  // Process each received CAN message
  for (auto const &can_frame : pi3hat_input.rx_can)
  {
    int can_id = can_frame.id;
    // If the id is -1, then it is the attitude message
    if (can_id == -1)
    {
      // Update the IMU observation
      mjbots::pi3hat::Euler euler_rad = quat_to_euler(pi3hat_input.attitude->attitude);
      robot_obs.imu_obs.roll = euler_rad.pitch;
      robot_obs.imu_obs.pitch = euler_rad.roll + M_PI / 2;

      robot_obs.imu_obs.roll_vel = pi3hat_input.attitude->rate_dps.z * M_PI / 180;
      robot_obs.imu_obs.pitch_vel = pi3hat_input.attitude->rate_dps.x * M_PI / 180;
      continue;
    }
    // Decode the actuator observation
    decode_actuator_observation(can_frame.data, robot_config, robot_obs);
  }
}

void HardwareInterface::enable()
{
  for (auto const &[name, config] : this->robot_config.actuator_configs)
  {
    int can_index = config.id - 1;
    pi3hat_input.tx_can[can_index].id = config.id;
    pi3hat_input.tx_can[can_index].bus = config.bus;
    pi3hat_input.tx_can[can_index].expect_reply = true;
    uint8_t *data = pi3hat_input.tx_can[can_index].data;
    // Encode the enable message
    for (int i = 0; i < 8; i++)
    {
      data[i] = enable_msg[i];
    }
  }
  // Update the Pi3Hat
  const auto result = pi3hat->Cycle(pi3hat_input);
}

void HardwareInterface::disable()
{
  for (auto const &[name, config] : this->robot_config.actuator_configs)
  {
    int can_index = config.id - 1;
    pi3hat_input.tx_can[can_index].id = config.id;
    pi3hat_input.tx_can[can_index].bus = config.bus;
    pi3hat_input.tx_can[can_index].expect_reply = true;
    uint8_t *data = pi3hat_input.tx_can[can_index].data;
    // Encode the disable message
    for (int i = 0; i < 8; i++)
    {
      data[i] = enable_msg[i];
    }
  }
  // Update the Pi3Hat
  const auto result = pi3hat->Cycle(pi3hat_input);
}

void HardwareInterface::zero()
{
  for (auto const &[name, config] : this->robot_config.actuator_configs)
  {
    int can_index = config.id - 1;
    pi3hat_input.tx_can[can_index].id = config.id;
    pi3hat_input.tx_can[can_index].bus = config.bus;
    pi3hat_input.tx_can[can_index].expect_reply = true;
    uint8_t *data = pi3hat_input.tx_can[can_index].data;
    // Encode the zero message
    for (int i = 0; i < 8; i++)
    {
      data[i] = enable_msg[i];
    }
  }
  // Update the Pi3Hat
  const auto result = pi3hat->Cycle(pi3hat_input);
}

mjbots::pi3hat::Euler quat_to_euler(mjbots::pi3hat::Quaternion q) {
  double w_ = q.w;
  double x_ = q.x;
  double y_ = q.y;
  double z_ = q.z;
  mjbots::pi3hat::Euler result_rad;

  const double sinp = 2.0 * (w_ * y_ - z_ * x_);
  if (sinp >= (1.0 - 1e-8)) {
    result_rad.pitch = M_PI / 2.0;
    result_rad.roll = 0.0;
    result_rad.yaw = -2.0 * atan2(x_, w_);
  } else if (sinp <= (-1.0 + 1e-8)) {
    result_rad.pitch = -M_PI / 2.0;
    result_rad.roll = 0.0;
    result_rad.yaw = 2.0 * atan2(x_, w_);
  } else {
    result_rad.pitch = asin(sinp);

    const double sinr_cosp = 2.0 * (w_ * x_ + y_ * z_);
    const double cosr_cosp = 1.0 - 2.0 * (x_ * x_ + y_ * y_);
    result_rad.roll = atan2(sinr_cosp, cosr_cosp);

    const double siny_cosp = 2.0 * (w_ * z_ + x_ * y_);
    const double cosy_cosp = 1.0 - 2.0 * (y_ * y_ + z_ * z_);
    result_rad.yaw = atan2(siny_cosp, cosy_cosp);
  }

  return result_rad;
}
