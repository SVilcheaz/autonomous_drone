"""Publish only sensor mounting transforms; the VIO/PX4 navigation TF comes later."""
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    nodes = []
    for side, y in (("left", "0.06"), ("right", "-0.06")):
        nodes.append(Node(
            package="tf2_ros", executable="static_transform_publisher",
            name=f"phase2_{side}_mount", output="screen",
            arguments=["--x", "0.25", "--y", y, "--z", "0.14",
                       "--roll", "-1.5707963267948966", "--pitch", "0",
                       "--yaw", "-1.5707963267948966", "--frame-id", "base_link",
                       "--child-frame-id", f"stereo_{side}_optical_frame"],
            parameters=[{"use_sim_time": True}]))
    return LaunchDescription(nodes)
