#!/usr/bin/env python3
"""Apply a slow left-right yaw sweep to an isolated SITL hover setpoint stream."""

import math

from px4_msgs.msg import TrajectorySetpoint
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, qos_profile_sensor_data
from rosgraph_msgs.msg import Clock
from std_msgs.msg import String


# PX4 NED yaw is positive clockwise. Time is measured on the simulator clock.
SWEEP = ((0.0, 0.0), (1.0, 0.0), (2.5, -20.0), (4.5, -20.0),
         (7.5, 20.0), (9.5, 20.0), (11.0, 0.0))


def yaw_offset(elapsed):
    for (start, first), (end, second) in zip(SWEEP, SWEEP[1:]):
        if start <= elapsed < end:
            return math.radians(first + (second - first) * (elapsed - start) / (end - start))
    return 0.0


class YawCheckRelay(Node):
    def __init__(self):
        super().__init__('phase3_yaw_check_relay')
        self.clock_ns = 0
        self.waypoint_origin = None
        self.in_waypoint = False
        self.finished_sweep = False
        self.output = self.create_publisher(TrajectorySetpoint, '/fmu/in/trajectory_setpoint', 10)
        self.create_subscription(Clock, '/clock', self.on_clock, qos_profile_sensor_data)
        self.create_subscription(String, '/mission/state', self.on_state,
                                 QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.create_subscription(TrajectorySetpoint, '/phase3/yaw_check/trajectory_setpoint',
                                 self.on_setpoint, 10)

    def on_clock(self, msg):
        current = msg.clock.sec * 1_000_000_000 + msg.clock.nanosec
        if current < self.clock_ns:
            self.waypoint_origin = None
            self.in_waypoint = False
        self.clock_ns = current

    def on_state(self, msg):
        self.in_waypoint = msg.data == 'WAYPOINT'
        if self.in_waypoint and self.waypoint_origin is None and self.clock_ns > 0:
            self.waypoint_origin = self.clock_ns
            self.get_logger().info('Starting the scheduled -20 and +20 degree yaw offsets')

    def on_setpoint(self, msg):
        if self.in_waypoint and self.waypoint_origin is not None and math.isfinite(msg.yaw):
            elapsed = (self.clock_ns - self.waypoint_origin) * 1e-9
            msg.yaw = math.remainder(msg.yaw + yaw_offset(elapsed), math.tau)
            if elapsed >= SWEEP[-1][0] and not self.finished_sweep:
                self.finished_sweep = True
                self.get_logger().info('Yaw sweep complete, returning to the controller heading')
        self.output.publish(msg)


def main():
    rclpy.init()
    node = YawCheckRelay()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
