// Copyright 2023 Universidad Politécnica de Madrid
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
//    * Redistributions of source code must retain the above copyright
//      notice, this list of conditions and the following disclaimer.
//
//    * Redistributions in binary form must reproduce the above copyright
//      notice, this list of conditions and the following disclaimer in the
//      documentation and/or other materials provided with the distribution.
//
//    * Neither the name of the Universidad Politécnica de Madrid nor the names of its
//      contributors may be used to endorse or promote products derived from
//      this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.

/**
 * @file pixhawk_platform.hpp
 *
 * PixhawkPlatform class definition
 *
 * @author Miguel Fernández Cortizas
 *         Rafael Pérez Seguí
 *         Pedro Arias Pérez
 *         Javier Melero Deza
 */

#ifndef AS2_PLATFORM_PIXHAWK__PIXHAWK_PLATFORM_HPP_
#define AS2_PLATFORM_PIXHAWK__PIXHAWK_PLATFORM_HPP_

#include <chrono>
#include <cmath>
#include <memory>
#include <string>

#include <px4_msgs/msg/battery_status.hpp>
#include <px4_msgs/msg/manual_control_switches.hpp>
#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <px4_msgs/msg/sensor_combined.hpp>
#include <px4_msgs/msg/sensor_gps.hpp>
#include <px4_msgs/msg/timesync_status.hpp>
#include <px4_msgs/msg/trajectory_setpoint.hpp>
#include <px4_msgs/msg/vehicle_attitude_setpoint.hpp>
#include <px4_msgs/msg/vehicle_command.hpp>
#include <px4_msgs/msg/vehicle_control_mode.hpp>
#include <px4_msgs/msg/vehicle_odometry.hpp>
#include <px4_msgs/msg/vehicle_rates_setpoint.hpp>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/battery_state.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <sensor_msgs/msg/nav_sat_status.hpp>
#include "as2_core/custom/tf2_geometry_msgs.hpp"

#include "as2_core/utils/frame_utils.hpp"
#include "as2_core/aerial_platform.hpp"
#include "as2_core/names/topics.hpp"
#include "as2_core/sensor.hpp"
#include "as2_core/utils/tf_utils.hpp"
#include "as2_msgs/msg/control_mode.hpp"
#include "as2_msgs/msg/thrust.hpp"

namespace as2_platform_pixhawk
{

class PixhawkPlatform : public as2::AerialPlatform
{
public:
  /**
   * @brief Construct the PX4 platform, creating the uORB interfaces.
   *
   * @param options Node options.
   */
  explicit PixhawkPlatform(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
  /**
   * @brief Destroy the Pixhawk Platform object.
   */
  ~PixhawkPlatform() {}

public:
  /**
   * @brief Create the sensor interfaces the platform publishes.
   */
  void configureSensors();
  /**
   * @brief Publish the sensor measurements read from the autopilot.
   */
  void publishSensorData();

  // TODO(miferco97): set ATTITUDE as default mode with yaw_speed = 0  and Thrust = 0 N
  /**
   * @brief Set the control mode the platform falls back to. Not implemented.
   */
  void setDefaultControlMode() {}

  /**
   * @brief Arm or disarm the vehicle.
   *
   * @param state True to arm, false to disarm.
   * @return true if the vehicle accepted the request.
   */
  bool ownSetArmingState(bool state);
  /**
   * @brief Enter or leave offboard control.
   *
   * @param offboard True to take control, false to release it.
   * @return true if the vehicle accepted the request.
   */
  bool ownSetOffboardControl(bool offboard);
  /**
   * @brief Accept a control mode requested through the platform interface.
   *
   * @param msg Requested control mode.
   * @return true if the platform accepts the mode.
   */
  bool ownSetPlatformControlMode(const as2_msgs::msg::ControlMode & msg);
  /**
   * @brief Send the actuator commands, keeping the offboard heartbeat alive
   * even while the platform is not sending references.
   */
  void sendCommand() override;
  /**
   * @brief Send the current actuator commands to the vehicle.
   *
   * @return true if the command was sent.
   */
  bool ownSendCommand();
  /**
   * @brief Stop the motors immediately, without landing.
   */
  void ownKillSwitch() override;
  /**
   * @brief Hold the vehicle in place with a zero setpoint.
   */
  void ownStopPlatform() override;

  /**
   * @brief Zero the trajectory setpoint sent to the autopilot.
   */
  void resetTrajectorySetpoint();

  /**
   * @brief Zero the attitude setpoint sent to the autopilot.
   */
  void resetAttitudeSetpoint();

  /**
   * @brief Zero the body rates setpoint sent to the autopilot.
   */
  void resetRatesSetpoint();

  /**
   * @brief Get whether the platform is running against a simulated autopilot.
   *
   * @return true in simulation mode.
   */
  bool getFlagSimulationMode();

private:
  bool has_mode_settled_ = false;

  std::unique_ptr<as2::sensors::Imu> imu_sensor_ptr_;
  std::unique_ptr<as2::sensors::Sensor<sensor_msgs::msg::BatteryState>> battery_sensor_ptr_;
  std::unique_ptr<as2::sensors::Sensor<nav_msgs::msg::Odometry>> odometry_raw_estimation_ptr_;
  std::unique_ptr<as2::sensors::GPS> gps_sensor_ptr_;

  rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr external_odometry_sub_;
  /**
   * @brief Forward an external velocity estimate to the autopilot, as visual
   * odometry.
   *
   * @param msg Twist of the vehicle, from an external localization system.
   */
  void externalOdomCb(const geometry_msgs::msg::TwistStamped::SharedPtr msg);

  // PX4 subscribers
  rclcpp::Subscription<px4_msgs::msg::SensorCombined>::SharedPtr px4_imu_sub_;
  rclcpp::Subscription<px4_msgs::msg::VehicleOdometry>::SharedPtr px4_odometry_sub_;
  rclcpp::Subscription<px4_msgs::msg::VehicleControlMode>::SharedPtr px4_vehicle_control_mode_sub_;
  rclcpp::Subscription<px4_msgs::msg::TimesyncStatus>::SharedPtr px4_timesync_sub_;
  rclcpp::Subscription<px4_msgs::msg::BatteryStatus>::SharedPtr px4_battery_sub_;
  rclcpp::Subscription<px4_msgs::msg::SensorGps>::SharedPtr px4_gps_sub_;

  // PX4 publishers
  rclcpp::Publisher<px4_msgs::msg::ManualControlSwitches>::SharedPtr
    px4_manual_control_switches_pub_;
  rclcpp::Publisher<px4_msgs::msg::OffboardControlMode>::SharedPtr px4_offboard_control_mode_pub_;
  rclcpp::Publisher<px4_msgs::msg::TrajectorySetpoint>::SharedPtr px4_trajectory_setpoint_pub_;
  rclcpp::Publisher<px4_msgs::msg::VehicleCommand>::SharedPtr px4_vehicle_command_pub_;
  rclcpp::Publisher<px4_msgs::msg::VehicleAttitudeSetpoint>::SharedPtr
    px4_vehicle_attitude_setpoint_pub_;
  rclcpp::Publisher<px4_msgs::msg::VehicleRatesSetpoint>::SharedPtr px4_vehicle_rates_setpoint_pub_;
  rclcpp::Publisher<px4_msgs::msg::VehicleOdometry>::SharedPtr px4_visual_odometry_pub_;

  // PX4 Functions
  /**
   * @brief Send the arm command to PX4.
   */
  void PX4arm();
  /**
   * @brief Send the disarm command to PX4.
   */
  void PX4disarm();
  /**
   * @brief Publish the offboard control mode heartbeat, which tells PX4 which
   * kind of setpoint it must expect.
   */
  void PX4publishOffboardControlMode();
  /**
   * @brief Publish the current trajectory setpoint to PX4.
   */
  void PX4publishTrajectorySetpoint();
  /**
   * @brief Publish the current attitude setpoint to PX4.
   */
  void PX4publishAttitudeSetpoint();
  /**
   * @brief Publish the current body rates setpoint to PX4.
   */
  void PX4publishRatesSetpoint();
  /**
   * @brief Send a vehicle command to PX4.
   *
   * @param command PX4 command id.
   * @param param1 First command parameter.
   * @param param2 Second command parameter.
   */
  void PX4publishVehicleCommand(uint16_t command, float param1 = 0.0, float param2 = 0.0);
  /**
   * @brief Publish the last visual odometry to the PX4 estimator.
   */
  void PX4publishVisualOdometry();

private:
  bool manual_from_operator_ = false;
  bool set_disarm_ = false;
  nav_msgs::msg::Odometry odometry_msg_;

  std::atomic<uint64_t> timestamp_;

  px4_msgs::msg::OffboardControlMode px4_offboard_control_mode_;
  px4_msgs::msg::TrajectorySetpoint px4_trajectory_setpoint_;
  px4_msgs::msg::VehicleAttitudeSetpoint px4_attitude_setpoint_;
  px4_msgs::msg::VehicleRatesSetpoint px4_rates_setpoint_;
  px4_msgs::msg::VehicleOdometry px4_visual_odometry_msg_;

  float max_thrust_;
  float min_thrust_;
  bool simulation_mode_ = false;
  bool external_odom_ = true;
  std::string base_link_frame_id_;
  std::string odom_frame_id_;
  int target_system_id_ = 1;

  Eigen::Quaterniond q_ned_to_enu_;
  Eigen::Quaterniond q_enu_to_ned_;
  Eigen::Quaterniond q_aircraft_to_baselink_;
  Eigen::Quaterniond q_baselink_to_aircraft_;

private:
  // PX4 Callbacks
  void px4imuCallback(const px4_msgs::msg::SensorCombined::SharedPtr msg);
  void px4odometryCallback(const px4_msgs::msg::VehicleOdometry::SharedPtr msg);
  void px4VehicleControlModeCallback(const px4_msgs::msg::VehicleControlMode::SharedPtr msg);
  void px4BatteryCallback(const px4_msgs::msg::BatteryStatus::SharedPtr msg);
  void px4GpsCallback(const px4_msgs::msg::SensorGps::SharedPtr msg);
};

}  // namespace as2_platform_pixhawk

#endif  // AS2_PLATFORM_PIXHAWK__PIXHAWK_PLATFORM_HPP_
