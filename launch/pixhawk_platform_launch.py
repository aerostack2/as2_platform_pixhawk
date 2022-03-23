import launch
from launch import LaunchDescription, conditions
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, EnvironmentVariable


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('drone_id', default_value='drone0'),
        DeclareLaunchArgument('mass', default_value='1.0'),
        DeclareLaunchArgument('simulation_mode', default_value='false'),
        DeclareLaunchArgument('max_thrust', default_value='0.0'),
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
                "max_thrust": LaunchConfiguration('max_thrust')
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
                "max_thrust": LaunchConfiguration('max_thrust')
                }],
            remappings=[("sensor_measurements/odometry", "self_localization/odom")],
            condition= launch.conditions.IfCondition(LaunchConfiguration("simulation_mode"))
        ),
    
    ])
