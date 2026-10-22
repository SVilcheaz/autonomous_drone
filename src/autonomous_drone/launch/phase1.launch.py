from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'params_file',
            default_value=PathJoinSubstitution([
                FindPackageShare('autonomous_drone'), 'config', 'phase1.yaml']),
            description='Mission YAML file'),
        DeclareLaunchArgument('auto_start', default_value='false'),
        Node(
            package='autonomous_drone', executable='waypoint_mission',
            name='waypoint_mission', output='screen',
            parameters=[LaunchConfiguration('params_file'), {
                'auto_start': ParameterValue(
                    LaunchConfiguration('auto_start'), value_type=bool),
                'use_sim_time': False,
            }]),
    ])
