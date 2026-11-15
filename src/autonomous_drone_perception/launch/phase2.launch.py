"""Already-rectified WSL rig -> CUDA SGM -> calibrated, validated metric depth."""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import ComposableNodeContainer, Node
from launch_ros.descriptions import ComposableNode
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    stereo_remaps = [(topic, PathJoinSubstitution([
        LaunchConfiguration('stereo_namespace'), topic])) for topic in (
            'left/image_rect', 'right/image_rect', 'left/camera_info', 'right/camera_info')]
    return LaunchDescription([
        DeclareLaunchArgument('params_file', default_value=PathJoinSubstitution([
            FindPackageShare('autonomous_drone_perception'), 'config', 'phase2.yaml'])),
        # Absolute VPI confidence: require about <=0.5 px left/right disagreement
        # with a 64 px search (the old 60000 allowed about 5.4 px).
        DeclareLaunchArgument('confidence_threshold', default_value='65024'),
        DeclareLaunchArgument('stereo_namespace', default_value='/stereo'),
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
                    'confidence_threshold': ParameterValue(
                        LaunchConfiguration('confidence_threshold'), value_type=int),
                    'confidence_type': 0,
                    'window_size': 7,
                    'input_qos': 'SENSOR_DATA',
                    'output_qos': 'SENSOR_DATA',
                    'input_queue_size': 5,
                    'output_queue_size': 2,
                    'memory_pool_block_size': 640 * 480 * 4,
                    'memory_pool_num_blocks': 8,
                }], remappings=stereo_remaps + [
                    ('disparity', '/perception/disparity_raw'),
                ], extra_arguments=[{'use_intra_process_comms': True}])]),
        Node(package='autonomous_drone_perception', executable='metric_depth',
             name='metric_depth', output='screen',
             parameters=[LaunchConfiguration('params_file')], remappings=stereo_remaps + [
                 ('disparity', '/perception/disparity_raw'),
                 ('depth', '/perception/depth'),
                 ('depth_fresh', '/perception/depth_fresh'),
             ]),
    ])
