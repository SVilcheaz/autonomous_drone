"""Exercise capture stamps, staleness, clock pauses and resets on isolated ROS DDS."""
import copy
import os
from pathlib import Path
import signal
import struct
import subprocess
import sys
import tempfile
import time
import unittest

import rclpy
from rclpy.qos import qos_profile_sensor_data
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import CameraInfo, Image
from stereo_msgs.msg import DisparityImage
from std_msgs.msg import Bool

BINARY = Path(sys.argv.pop(1))


class MetricDepthTransport(unittest.TestCase):
    def setUp(self):
        self.assertEqual(os.environ.get('ROS_DOMAIN_ID'), '72')
        self.log = tempfile.TemporaryFile(mode='w+')
        self.process = subprocess.Popen([
            str(BINARY), '--ros-args', '-r', '__ns:=/phase2_contract_test',
            '-p', 'use_sim_time:=true', '-p', 'max_capture_age_s:=0.25',
            '-p', 'receipt_timeout_s:=0.25'], stdout=self.log, stderr=subprocess.STDOUT)
        rclpy.init()
        self.node = rclpy.create_node('metric_depth_transport_test')
        self.clock_pub = self.node.create_publisher(Clock, '/clock', 10)
        self.left_pub = self.node.create_publisher(CameraInfo, '/phase2_contract_test/left/camera_info', qos_profile_sensor_data)
        self.right_pub = self.node.create_publisher(CameraInfo, '/phase2_contract_test/right/camera_info', qos_profile_sensor_data)
        self.disparity_pub = self.node.create_publisher(DisparityImage, '/phase2_contract_test/disparity', qos_profile_sensor_data)
        self.depths, self.fresh = [], []
        self.subs = [
            self.node.create_subscription(Image, '/phase2_contract_test/depth', self.depths.append, qos_profile_sensor_data),
            self.node.create_subscription(Bool, '/phase2_contract_test/depth_fresh', lambda m: self.fresh.append(m.data), 10)]
        self.now_ns = 100_000_000_000
        self.pump(1.0, frames=False)

    def tearDown(self):
        self.node.destroy_node()
        rclpy.shutdown()
        self.process.send_signal(signal.SIGINT)
        try:
            self.process.wait(timeout=5)
        except subprocess.TimeoutExpired:
            self.process.kill()
            self.process.wait(timeout=5)
        self.log.seek(0)
        print(self.log.read())
        self.log.close()

    def pump(self, duration, frames=True, clock=True, offset_ns=0):
        end = time.monotonic() + duration
        while time.monotonic() < end:
            if clock:
                self.now_ns += 20_000_000
                tick = Clock()
                tick.clock.sec, tick.clock.nanosec = divmod(self.now_ns, 1_000_000_000)
                self.clock_pub.publish(tick)
            rclpy.spin_once(self.node, timeout_sec=0.01)
            if frames:
                left = CameraInfo()
                left.header.stamp.sec, left.header.stamp.nanosec = divmod(self.now_ns + offset_ns, 1_000_000_000)
                left.header.frame_id = 'stereo_left_optical_frame'
                left.width, left.height = 8, 1
                left.r = [1., 0., 0., 0., 1., 0., 0., 0., 1.]
                left.p = [400., 0., 4., 0., 0., 400., 0., 0., 0., 0., 1., 0.]
                right = copy.deepcopy(left)
                right.header.frame_id = 'stereo_right_optical_frame'
                right.p[3] = -48.
                disparity = DisparityImage()
                disparity.header = copy.deepcopy(right.header)
                disparity.f, disparity.t = 400., 48.
                disparity.max_disparity = 64.
                disparity.image.width, disparity.image.height = 8, 1
                disparity.image.encoding = '32FC1'
                disparity.image.step = 32
                disparity.image.data = struct.pack('<8f', 24, 24, 24, 0, -1, float('nan'), float('inf'), 80)
                self.left_pub.publish(left)
                self.right_pub.publish(right)
                self.disparity_pub.publish(disparity)
            stop = time.monotonic() + .02
            while time.monotonic() < stop:
                rclpy.spin_once(self.node, timeout_sec=.005)

    def test_capture_freshness_and_clock_reset(self):
        self.pump(1.0)
        self.assertGreater(len(self.depths), 3)
        self.assertTrue(self.fresh[-1])
        depth = self.depths[-1]
        self.assertEqual(depth.header.frame_id, 'stereo_left_optical_frame')
        values = struct.unpack('<8f', bytes(depth.data))
        self.assertEqual(values[:3], (2., 2., 2.))
        self.assertTrue(all(x != x for x in values[3:]))
        self.assertLessEqual(abs(self.now_ns - (depth.header.stamp.sec * 10**9 + depth.header.stamp.nanosec)), 40_000_000)

        self.pump(.15, frames=False)
        count = len(self.depths)
        self.pump(.5, frames=False)
        self.assertFalse(self.fresh[-1])
        self.assertEqual(len(self.depths), count, 'source loss must not republish depth')
        self.pump(.5, offset_ns=-1_000_000_000)
        self.assertEqual(len(self.depths), count, 'stale capture stamps must be rejected')

        self.pump(.5)
        self.assertTrue(self.fresh[-1])
        self.pump(.15, frames=False, clock=False)
        count = len(self.depths)
        self.pump(.5, frames=False, clock=False)
        self.assertFalse(self.fresh[-1], 'steady watchdog must expire even when /clock pauses')
        self.assertEqual(len(self.depths), count)

        self.now_ns = 1_000_000_000
        self.pump(.3, frames=False)
        self.assertFalse(self.fresh[-1])
        self.pump(.8)
        self.assertTrue(self.fresh[-1], 'new epoch must accept fresh depth after a clock reset')
        self.assertLess(self.depths[-1].header.stamp.sec, 10)


if __name__ == '__main__':
    unittest.main()
