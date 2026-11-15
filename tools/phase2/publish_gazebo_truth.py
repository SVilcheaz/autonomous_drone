#!/usr/bin/env python3
"""WSL-only evaluation bridge retaining Gazebo pose names and simulation stamps.

Requires the existing ROS environment and Gazebo CLI, without Python Gazebo
bindings. Never publishes /tf, /tf_static, or operational perception topics.
"""
import argparse
import json
import subprocess
import time

import rclpy
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import String


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--world', default='phase2_validation')
    args = parser.parse_args()
    rclpy.init()
    node = rclpy.create_node('phase2_gazebo_truth_evaluation_only')
    publisher = node.create_publisher(String, '/evaluation/gazebo_poses',
                                      qos_profile_sensor_data)
    process = subprocess.Popen(
        ['gz', 'topic', '-e', '-t', f'/world/{args.world}/pose/info', '--json-output'],
        stdout=subprocess.PIPE, text=True, bufsize=1)
    decoder, pending, last = json.JSONDecoder(), '', 0.0
    count = 0
    try:
        for line in process.stdout:
            pending += line
            if len(pending) > 4 * 1024 * 1024:
                raise RuntimeError('Gazebo output is not a bounded JSON pose stream')
            while pending.strip():
                pending = pending.lstrip()
                try:
                    message, end = decoder.raw_decode(pending)
                except json.JSONDecodeError:
                    break
                pending = pending[end:]
                if 'pose' not in message or 'header' not in message:
                    raise RuntimeError('Gazebo pose message lacks pose/header fields')
                now = time.monotonic()
                if now - last >= .05:
                    publisher.publish(String(data=json.dumps(message, separators=(',', ':'))))
                    last = now
                    count += 1
                    if count == 1 or count % 200 == 0:
                        print(f'Evaluation-only truth messages published: {count}', flush=True)
                rclpy.spin_once(node, timeout_sec=0)
        raise RuntimeError(f'Gazebo pose reader exited: {process.wait()}')
    except KeyboardInterrupt:
        pass
    finally:
        process.terminate()
        try:
            process.wait(timeout=3)
        except subprocess.TimeoutExpired:
            process.kill()
            process.wait()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
