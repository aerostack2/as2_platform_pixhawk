#!/bin/bash
ros2 launch pixhawk_platform pixhawk_platform_launch.py drone_id:='drone_1' config:=config/platform_default.yaml  simulation_mode:=true use_sim_time:=false
