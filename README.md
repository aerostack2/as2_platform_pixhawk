# as2_platform_pixhawk


[Aerostack2](https://aerostack2.github.io/) Aerial platform for the PX4 autopilot

For a complete installation guide follow [PX4 instructions](https://aerostack2.github.io/_03_aerial_platforms/_pixhawk/index.html#installation)




## Multiple PX4 Operation modes:

![image info](./docs/pixhawk_odometry.png)


## PX4 Msgs:

### Availables:

* /fmu/in/obstacle_distance
* /fmu/in/offboard_control_mode
* /fmu/in/onboard_computer_status
* /fmu/in/sensor_optical_flow
* /fmu/in/telemetry_status
* /fmu/in/trajectory_setpoint
* /fmu/in/vehicle_attitude_setpoint
* /fmu/in/vehicle_command
* /fmu/in/vehicle_mocap_odometry
* /fmu/in/vehicle_rates_setpoint
* /fmu/in/vehicle_trajectory_bezier
* /fmu/in/vehicle_trajectory_waypoint
* /fmu/in/vehicle_visual_odometry
* /fmu/out/failsafe_flags
* /fmu/out/sensor_combined
* /fmu/out/timesync_status
* /fmu/out/vehicle_attitude
* /fmu/out/vehicle_control_mode
* /fmu/out/vehicle_global_position
* /fmu/out/vehicle_gps_position
* /fmu/out/vehicle_local_position
* /fmu/out/vehicle_odometry
* /fmu/out/vehicle_status


### Used:

* IMU: 
/fmu/out/sensor_combined
* Set Control Mode: 
/fmu/out/vehicle_control_mode
* Get GPS: 
/fmu/out/vehicle_gps_position
* Get position: 
/fmu/out/vehicle_odometry

* Set control Mode: 
/fmu/in/offboard_control_mode
* Set trajectory reference: 
/fmu/in/trajectory_setpoint
* Set attitude reference: 
/fmu/in/vehicle_attitude_setpoint
* Set rate reference: 
/fmu/in/vehicle_rates_setpoint
* Send vehicle command for arm: 
/fmu/in/vehicle_command
* Send vehicle visual odometry: 
/fmu/in/vehicle_visual_odometry


### Not Availables:

* Get battery status: 
"/fmu/out/battery_status"
* Kill switch: 
fmu/in/manual_control_switches


### Not Used:

* /fmu/in/obstacle_distance
* /fmu/in/onboard_computer_status
* /fmu/in/sensor_optical_flow
* /fmu/in/telemetry_status
* /fmu/in/vehicle_mocap_odometry
* /fmu/in/vehicle_trajectory_bezier
* /fmu/in/vehicle_trajectory_waypoint
* /fmu/out/failsafe_flags
* /fmu/out/timesync_status
* /fmu/out/vehicle_attitude
* /fmu/out/vehicle_control_mode
* /fmu/out/vehicle_global_position
* /fmu/out/vehicle_local_position
* /fmu/out/vehicle_status
