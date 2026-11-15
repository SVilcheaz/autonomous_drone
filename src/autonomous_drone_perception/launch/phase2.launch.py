"""Already-rectified WSL rig -> CUDA SGM -> calibrated, validated metric depth."""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import ComposableNodeContainer, Node
from launch_ros.descriptions import ComposableNode
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('params_file', default_value=PathJoinSubstitution([
            FindPackageShare('autonomous_drone_perception'), 'config', 'phase2.yaml'])),
        ComposableNodeContainer(
            name='stereo_container', namespace='',
            package='rclcpp_components', executable='component_container',
            output='screen', composable_node_descriptions=[ComposableNode(
                package='isaac_ros_stereo_image_proc',
                plugin='nvidia::isaac_ros::stereo_image_proc::DisparityNode',
                name='sgm', parameters=[{
                    'use_sim_time': True,
                    'backend': 'CUDA',
                    'max_disparity': 64.0,
                    'confidence_threshold': 60000,
                    'confidence_type': 0,
                    'window_size': 7,
                    'input_qos': 'SENSOR_DATA',
                    'output_qos': 'SENSOR_DATA',
                    'input_queue_size': 5,
                    'output_queue_size': 2,
                    'memory_pool_block_size': 640 * 480 * 4,
                    'memory_pool_num_blocks': 8,
                }], remappings=[
                    ('left/image_rect', '/stereo/left/image_rect'),
                    ('right/image_rect', '/stereo/right/image_rect'),
                    ('left/camera_info', '/stereo/left/camera_info'),
                    ('right/camera_info', '/stereo/right/camera_info'),
                    ('disparity', '/perception/disparity_raw'),
                ], extra_arguments=[{'use_intra_process_comms': True}])]),
        Node(package='autonomous_drone_perception', executable='metric_depth',
             name='metric_depth', output='screen',
             parameters=[LaunchConfiguration('params_file')], remappings=[
                 ('disparity', '/perception/disparity_raw'),
                 ('left/camera_info', '/stereo/left/camera_info'),
                 ('right/camera_info', '/stereo/right/camera_info'),
                 ('depth', '/perception/depth'),
                 ('depth_fresh', '/perception/depth_fresh'),
             ]),
    ])
