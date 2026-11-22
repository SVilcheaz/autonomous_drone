from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode


def generate_launch_description():
    stereo = LaunchConfiguration('stereo_namespace')
    visual_odometry = ComposableNode(
        package='isaac_ros_visual_slam',
        plugin='nvidia::isaac_ros::visual_slam::VisualSlamNode',
        name='visual_slam',
        parameters=[{
            'use_sim_time': True,
            'tracking_mode': 1,
            'num_cameras': 2,
            'min_num_images': 2,
            'rectified_images': True,
            'enable_localization_n_mapping': False,
            'enable_ground_constraint_in_odometry': False,
            'publish_map_to_odom_tf': False,
            'publish_odom_to_base_tf': True,
            'override_publishing_stamp': False,
            'sync_matching_threshold_ms': 1.0,
            # Nominal image capture period is 50 ms in simulation time.
            'image_jitter_threshold_ms': 55.0,
            'base_frame': 'base_link',
            'odom_frame': 'odom',
            'imu_frame': 'base_link',
            'camera_optical_frames': [
                'stereo_left_optical_frame', 'stereo_right_optical_frame',
            ],
            'calibration_frequency': 250.0,
            'image_qos': 'SENSOR_DATA',
            'imu_qos': 'SENSOR_DATA',
        }],
        remappings=[
            ('visual_slam/image_0', [stereo, '/left/image_rect']),
            ('visual_slam/image_1', [stereo, '/right/image_rect']),
            ('visual_slam/camera_info_0', [stereo, '/left/camera_info']),
            ('visual_slam/camera_info_1', [stereo, '/right/camera_info']),
            ('visual_slam/imu', '/imu/data'),
        ],
    )
    return LaunchDescription([
        DeclareLaunchArgument('stereo_namespace', default_value='/stereo'),
        ComposableNodeContainer(
            name='stereo_vio_container',
            namespace='',
            package='rclcpp_components',
            executable='component_container_mt',
            composable_node_descriptions=[visual_odometry],
            output='screen',
        ),
    ])
