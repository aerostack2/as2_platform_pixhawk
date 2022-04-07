from os.path import join
import launch
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PythonExpression
from ament_index_python.packages import get_package_share_directory


def join_config_files(config):
    with open(config, 'r') as reader:
        test = reader.read()

    if test[-1] != '\n':
        test += '\n'
    
    with open('/tmp/pixhawk_platform.yaml', 'w') as writer:
        writer.write(test)


def get_simultaion_flag(config):
    with open(config, 'r') as stream:
        for line in stream:
            if 'simulation_mode' in line:
                flag = line.split(':')[-1].strip()
                break
    
    return 'True' if str(flag).lower() == 'true' else 'False'


def generate_launch_description():
    config = join(
        get_package_share_directory('pixhawk_platform'),
        'config',
        'pixhawk_platform.yaml'
    )

    join_config_files(config)
    sim_mode = get_simultaion_flag(config)

    # if is not in simulation
    node_real = Node(
        package="pixhawk_platform",
        executable="pixhawk_platform_node",
        name="platform",
        namespace=LaunchConfiguration('drone_id'),
        output="screen",
        emulate_tty=True,
        parameters=[config],
        condition= launch.conditions.UnlessCondition(PythonExpression([sim_mode]))
    )
    # if is in simulation
    node_sim = Node(
        package="pixhawk_platform",
        executable="pixhawk_platform_node",
        name="platform",
        namespace=LaunchConfiguration('drone_id'),
        output="screen",
        emulate_tty=True,
        parameters=[config],
        remappings=[("sensor_measurements/odometry", "self_localization/odom")],
        condition= launch.conditions.IfCondition(PythonExpression([sim_mode]))
    )

    return LaunchDescription([
        DeclareLaunchArgument('drone_id', default_value='drone0'),
        DeclareLaunchArgument('mass', default_value='1.0'),
        DeclareLaunchArgument('max_thrust', default_value='0.0'),
        DeclareLaunchArgument('simulation_mode', default_value='false'),
        node_real,
        node_sim
    ])
