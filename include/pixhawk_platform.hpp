// "Copyright [year] <Copyright Owner>"

#ifndef PIXHAWK_PLATFORM_HPP_
#define PIXHAWK_PLATFORM_HPP_

#include <px4_ros_com/frame_transforms.h>

#include <chrono>
#include <cmath>
#include <memory>
#include <px4_msgs/msg/actuator_controls.hpp>
#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <px4_msgs/msg/sensor_combined.hpp>
#include <px4_msgs/msg/timesync.hpp>
#include <px4_msgs/msg/trajectory_setpoint.hpp>
#include <px4_msgs/msg/vehicle_attitude_setpoint.hpp>
#include <px4_msgs/msg/vehicle_command.hpp>
#include <px4_msgs/msg/vehicle_control_mode.hpp>
#include <px4_msgs/msg/vehicle_local_position_setpoint.hpp>
#include <px4_msgs/msg/vehicle_odometry.hpp>
#include <px4_msgs/msg/vehicle_rates_setpoint.hpp>
#include <px4_msgs/msg/vehicle_status.hpp>
#include <px4_msgs/msg/vehicle_visual_odometry.hpp>
#include <string>

#include "as2_core/aerial_platform.hpp"
#include "as2_core/sensor.hpp"
#include "as2_msgs/msg/thrust.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "sensor_msgs/msg/battery_state.hpp"
#include "sensor_msgs/msg/imu.hpp"

class PixhawkPlatform : public as2::AerialPlatform
{
public:
  PixhawkPlatform();

  void configureSensors();
  void publishSensorData();

  //TODO: set ATTITUDE as default mode with yaw_speed = 0  and Thrust = 0 N
  void setDefaultControlMode(){};

  bool ownSetArmingState(bool state);
  bool ownSetOffboardControl(bool offboard);
  bool ownSetPlatformControlMode(const as2_msgs::msg::PlatformControlMode & msg);
  bool ownSendCommand();

  void resetTrajectorySetpoint();
  void resetAttitudeSetpoint();
  void resetRatesSetpoint();

private:
  bool has_mode_settled_ = false;

  std::unique_ptr<as2::sensors::Imu> imu_sensor_ptr_;
  std::unique_ptr<as2::sensors::Sensor<sensor_msgs::msg::BatteryState>> battery_sensor_ptr_;
  std::unique_ptr<as2::sensors::Sensor<nav_msgs::msg::Odometry>> odometry_raw_estimation_ptr_;

  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odometry_sub_;

  // PX4 subscribers
  rclcpp::Subscription<px4_msgs::msg::SensorCombined>::SharedPtr px4_imu_sub_;
  rclcpp::Subscription<px4_msgs::msg::VehicleOdometry>::SharedPtr px4_odometry_sub_;
  rclcpp::Subscription<px4_msgs::msg::VehicleControlMode>::SharedPtr px4_vehicle_control_mode_sub_;
  rclcpp::Subscription<px4_msgs::msg::Timesync>::SharedPtr px4_timesync_sub_;

  // rclcpp::Subscription<px4_msgs::msg::BatteryState>::SharedPtr battery_sub_;

  // PX4 publishers
  rclcpp::Publisher<px4_msgs::msg::OffboardControlMode>::SharedPtr px4_offboard_control_mode_pub_;
  rclcpp::Publisher<px4_msgs::msg::TrajectorySetpoint>::SharedPtr px4_trajectory_setpoint_pub_;
  rclcpp::Publisher<px4_msgs::msg::VehicleCommand>::SharedPtr px4_vehicle_command_pub_;
  rclcpp::Publisher<px4_msgs::msg::VehicleAttitudeSetpoint>::SharedPtr
    px4_vehicle_attitude_setpoint_pub_;

  rclcpp::Publisher<px4_msgs::msg::VehicleRatesSetpoint>::SharedPtr px4_vehicle_rates_setpoint_pub_;

  rclcpp::Publisher<px4_msgs::msg::VehicleVisualOdometry>::SharedPtr px4_visual_odometry_pub_;

  // PX4 Functions
  void PX4arm() const;
  void PX4disarm() const;
  void PX4publishOffboardControlMode();
  void PX4publishTrajectorySetpoint();
  void PX4publishAttitudeSetpoint();
  void PX4publishRatesSetpoint();
  void PX4publishVehicleCommand(uint16_t command, float param1 = 0.0, float param2 = 0.0) const;
  void PX4publishVisualOdometry();

private:
  bool command_changes_ = false;
  sensor_msgs::msg::Imu imu_msg_;
  sensor_msgs::msg::BatteryState battery_msg_;
  nav_msgs::msg::Odometry px4_odometry_msg_;
  nav_msgs::msg::Odometry odometry_msg_;

  std::atomic<uint64_t> timestamp_;
  rclcpp::TimerBase::SharedPtr timer_;

  // as2_msgs::msg::PlatformControlMode platform_control_mode_;

  px4_msgs::msg::VehicleControlMode px4_control_mode_;
  px4_msgs::msg::VehicleStatus px4_vehicle_status_;
  px4_msgs::msg::OffboardControlMode px4_offboard_control_mode_;
  px4_msgs::msg::TrajectorySetpoint px4_trajectory_setpoint_;
  px4_msgs::msg::VehicleAttitudeSetpoint px4_attitude_setpoint_;
  px4_msgs::msg::VehicleRatesSetpoint px4_rates_setpoint_;
  px4_msgs::msg::VehicleVisualOdometry px4_visual_odometry_msg_;

private:
  void px4imuCallback(const px4_msgs::msg::SensorCombined::SharedPtr msg);
  void px4odometryCallback(const px4_msgs::msg::VehicleOdometry::SharedPtr msg);
  void px4VehicleControlModeCallback(const px4_msgs::msg::VehicleControlMode::SharedPtr msg);

  // void batteryCallback(const px4_msgs::msg::BatteryState::SharedPtr msg);
};

#endif  // PIXHAWK_TEST_HPP_
