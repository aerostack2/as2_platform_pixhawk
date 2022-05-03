from os.path import join

import launch
from launch import LaunchDescription, conditions
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, EnvironmentVariable
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    config = join(
        get_package_share_directory('pixhawk_platform'),
        'config',
        'control_modes.yaml'
    )
    return LaunchDescription([
        DeclareLaunchArgument('drone_id', default_value='drone0'),
        DeclareLaunchArgument('mass', default_value='1.0'),
        DeclareLaunchArgument('simulation_mode', default_value='false'),
        DeclareLaunchArgument('max_thrust', default_value='0.0'),
        DeclareLaunchArgument('control_modes_file', default_value=config),
        # if is not in simulation
        Node(
            package="pixhawk_platform",
            executable="pixhawk_platform_node",
            name="platform",
            namespace=LaunchConfiguration('drone_id'),
            output="screen",
            emulate_tty=True,
            parameters=[
                {"mass": LaunchConfiguration('mass'),
                "simulation_mode": LaunchConfiguration('simulation_mode'),
                "max_thrust": LaunchConfiguration('max_thrust'),
                "control_modes_file": LaunchConfiguration('control_modes_file')
                }],
            condition= launch.conditions.UnlessCondition(LaunchConfiguration("simulation_mode"))
        ),
        # if is in simulation
        Node(
            package="pixhawk_platform",
            executable="pixhawk_platform_node",
            name="platform",
            namespace=LaunchConfiguration('drone_id'),
            output="screen",
            emulate_tty=True,
            parameters=[
                {"mass": LaunchConfiguration('mass'),
                "simulation_mode": LaunchConfiguration('simulation_mode'),
                "max_thrust": LaunchConfiguration('max_thrust'),
                "control_modes_file": LaunchConfiguration('control_modes_file')
                }],
            remappings=[("sensor_measurements/odometry", "self_localization/odom")],
            condition= launch.conditions.IfCondition(LaunchConfiguration("simulation_mode"))
        ),
    
    ])
