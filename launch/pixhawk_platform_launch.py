from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration, EnvironmentVariable, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch.conditions import IfCondition

import yaml
from os.path import join
from ament_index_python.packages import get_package_share_directory

def get_platform_node(context, *args, **kargs):
    config = LaunchConfiguration('config').perform(context)

    with open(config, "r") as f:
        config_params = yaml.safe_load(f)

    try:
        control_modes_file = config_params["/**"]["ros__parameters"]["control_modes_file"]
    except KeyError:
        control_modes_file = ""

    if not control_modes_file:
        control_modes_file = join(
            get_package_share_directory('pixhawk_platform'),
            'config', 'control_modes.yaml'
        )

    dict = {'/**': {'ros__parameters': {'control_modes_file': f'{control_modes_file}'}
            }}
    with open('/tmp/aux_config.yaml', 'w') as f:
        yaml.dump(dict, f, default_flow_style=False)

    # if is in simulation
    node = Node(
        package="pixhawk_platform",
        executable="pixhawk_platform_node",
        name="platform",
        namespace=LaunchConfiguration('drone_id'),
        output="screen",
        emulate_tty=True,
        parameters=[config, 
                    '/tmp/aux_config.yaml',
                    {'use_sim_time': LaunchConfiguration('use_sim_time')}],
    )

    return [node]


def generate_launch_description():
    config = PathJoinSubstitution([
        FindPackageShare('pixhawk_platform'),
        'config', 'platform_default.yaml'
    ])

    return LaunchDescription([
        DeclareLaunchArgument('drone_id', default_value=EnvironmentVariable('AEROSTACK2_SIMULATION_DRONE_ID')),
        DeclareLaunchArgument('config', default_value=config),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        OpaqueFunction(function=get_platform_node)
    ])
