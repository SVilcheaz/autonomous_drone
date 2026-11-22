#!/usr/bin/env python3
"""Record the disarmed SITL ground check for EKF2 external-vision aiding."""

import argparse
import json
import math
from pathlib import Path
import statistics
import time

from px4_msgs.msg import (
    EstimatorStatusFlags, VehicleLandDetected, VehicleLocalPosition, VehicleOdometry, VehicleStatus,
)
import rclpy
from rclpy.qos import qos_profile_sensor_data
from rosgraph_msgs.msg import Clock


def fmu_topic(msg_type, path):
    version = getattr(msg_type, 'MESSAGE_VERSION', 0)
    return f'/fmu/{path}' + (f'_v{version}' if version else '')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--seconds', type=float, default=15.0)
    parser.add_argument('--gnss', choices=('active', 'disabled'), default='active')
    parser.add_argument('--vision-height', action='store_true')
    parser.add_argument('--output', type=Path)
    args = parser.parse_args()
    if not math.isfinite(args.seconds) or args.seconds < 5.0:
        parser.error('--seconds must be at least 5')

    rclpy.init()
    node = rclpy.create_node('phase3_ekf2_ground_check')
    clock_us = None
    vision = []
    flags = []
    status = []
    position = []
    land = []

    def on_clock(msg):
        nonlocal clock_us
        clock_us = msg.clock.sec * 1_000_000 + msg.clock.nanosec // 1000

    def on_vision(msg):
        if clock_us is not None:
            vision.append((msg, clock_us, time.monotonic()))

    node.create_subscription(Clock, '/clock', on_clock, qos_profile_sensor_data)
    for msg_type, path, callback in (
        (VehicleOdometry, 'in/vehicle_visual_odometry', on_vision),
        (EstimatorStatusFlags, 'out/estimator_status_flags', flags.append),
        (VehicleStatus, 'out/vehicle_status', status.append),
        (VehicleLocalPosition, 'out/vehicle_local_position', position.append),
        (VehicleLandDetected, 'out/vehicle_land_detected', land.append),
    ):
        node.create_subscription(msg_type, fmu_topic(msg_type, path), callback, qos_profile_sensor_data)

    def spin_for(seconds):
        stop = time.monotonic() + seconds
        while time.monotonic() < stop:
            rclpy.spin_once(node, timeout_sec=0.05)

    try:
        spin_for(4.0)
        for samples in (vision, flags, status, position, land):
            samples.clear()
        spin_for(args.seconds)
    finally:
        node.destroy_node()
        rclpy.shutdown()

    if not all((vision, flags, status, position, land)) or len(vision) < 2:
        report = {'passed': False, 'error': 'Incomplete live SITL streams', 'sample_counts': {
            'vision': len(vision), 'flags': len(flags), 'status': len(status),
            'position': len(position), 'land': len(land),
        }}
    else:
        captures = [msg.timestamp_sample for msg, _, _ in vision]
        gaps = [b - a for a, b in zip(captures, captures[1:])]
        capture_age = [clock - msg.timestamp_sample for msg, clock, _ in vision]
        valid = [msg.xy_valid and msg.z_valid and msg.v_xy_valid and msg.v_z_valid
                 and all(math.isfinite(value) for value in (msg.x, msg.y, msg.z, msg.heading))
                 for msg in position]
        disarmed = all(msg.arming_state == VehicleStatus.ARMING_STATE_DISARMED for msg in status)
        landed = all(msg.landed for msg in land)
        preflight = status[-1].pre_flight_checks_pass
        ev_pos = sum(msg.cs_ev_pos for msg in flags)
        ev_hgt = sum(msg.cs_ev_hgt for msg in flags)
        gnss_pos = sum(msg.cs_gnss_pos for msg in flags)
        gnss_vel = sum(msg.cs_gnss_vel for msg in flags)
        gnss_hgt = sum(msg.cs_gps_hgt for msg in flags)
        gnss_yaw = sum(msg.cs_gnss_yaw for msg in flags)
        gnss_as_expected = (gnss_pos == len(flags) if args.gnss == 'active'
                            else gnss_pos == gnss_vel == gnss_hgt == gnss_yaw == 0)
        report = {
            'passed': bool(disarmed and landed and all(valid) and len(flags) >= 3
                           and ev_pos == len(flags) and gnss_as_expected and min(gaps) > 0
                           and (not args.vision_height or ev_hgt == len(flags))),
            'scope': 'disarmed SITL telemetry and Jetson external-vision output',
            'expected_gnss': args.gnss, 'px4_disarmed': disarmed, 'px4_landed': landed,
            'pre_flight_checks_pass': preflight,
            'flags_samples': len(flags), 'ev_position_flag_samples': ev_pos,
            'ev_height_flag_samples': ev_hgt,
            'gnss_position_flag_samples': gnss_pos, 'gnss_velocity_flag_samples': gnss_vel,
            'gnss_height_flag_samples': gnss_hgt, 'gnss_yaw_flag_samples': gnss_yaw,
            'local_state_samples': len(position), 'valid_local_state_samples': sum(valid),
            'vision_samples': len(vision),
            'vision_capture_rate_hz': (len(captures) - 1) * 1e6 / (captures[-1] - captures[0])
            if captures[-1] > captures[0] else None,
            'vision_receipt_rate_hz': (len(vision) - 1) / (vision[-1][2] - vision[0][2]),
            'vision_capture_max_gap_us': max(gaps),
            'vision_capture_gaps_at_or_above_ekf2_interval': sum(gap >= 200_000 for gap in gaps),
            'vision_capture_age_median_us': statistics.median(capture_age),
            'vision_capture_age_max_us': max(capture_age),
            'vision_reset_counters': sorted({msg.reset_counter for msg, _, _ in vision}),
        }

    encoded = json.dumps(report, indent=2) + '\n'
    if args.output:
        args.output.write_text(encoded)
    print(encoded, end='')
    return not report['passed']


if __name__ == '__main__':
    raise SystemExit(main())
