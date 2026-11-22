#!/usr/bin/env python3
"""Measure live cuVSLAM output, tracking state, and capture-time alignment."""

import argparse
from bisect import bisect_left
from collections import Counter
import json
import math
import time

from isaac_ros_visual_slam_interfaces.msg import VisualSlamStatus
from nav_msgs.msg import Odometry
import numpy as np
import rclpy
from rclpy.qos import qos_profile_sensor_data
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import CameraInfo
from std_msgs.msg import String
from tf2_msgs.msg import TFMessage


def stamp_ns(stamp):
    return stamp.sec * 1_000_000_000 + stamp.nanosec


def xyz(vector):
    return vector.x, vector.y, vector.z


def p95(values):
    if values:
        return sorted(values)[math.ceil(0.95 * len(values)) - 1]
    return None


def quaternion_product(a, b):
    return np.append(a[3] * b[:3] + b[3] * a[:3] + np.cross(a[:3], b[:3]),
                     a[3] * b[3] - np.dot(a[:3], b[:3]))


def quaternion_conjugate(q):
    return q * [-1, -1, -1, 1]


def rotate(q, vector):
    return quaternion_product(quaternion_product(q, np.append(vector, 0)),
                              quaternion_conjugate(q))[:3]


def yaw(q):
    x, y, z, w = q
    return math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))


def truth_errors(truth, stamps, positions, orientations):
    truth_stamps = sorted(truth)
    matched = []
    for stamp, position, orientation in zip(stamps, positions, orientations):
        index = bisect_left(truth_stamps, stamp)
        if index == 0 or index == len(truth_stamps):
            continue
        before, after = truth_stamps[index - 1:index + 1]
        if after - before > 150_000_000:
            continue
        fraction = (stamp - before) / (after - before)
        p0, q0 = truth[before]
        p1, q1 = truth[after]
        if np.dot(q0, q1) < 0:
            q1 = -q1
        tp = p0 + fraction * (p1 - p0)
        tq = q0 + fraction * (q1 - q0)
        vq = np.array([orientation.x, orientation.y, orientation.z, orientation.w])
        matched.append((tp, tq / np.linalg.norm(tq), position, vq))

    if len(matched) < 2:
        return {'gazebo_truth_matches': len(matched)}

    tp0, tq0, vp0, vq0 = matched[0]
    alignment = quaternion_product(tq0, quaternion_conjugate(vq0))
    translation_error, angle_error, truth_displacement = [], [], []
    truth_yaw, vio_yaw, yaw_error = [], [], []
    initial_yaw = yaw(tq0)
    for tp, tq, vp, vq in matched:
        truth_delta = tp - tp0
        vio_delta = rotate(alignment, np.asarray(vp) - vp0)
        translation_error.append(math.dist(truth_delta, vio_delta))
        truth_displacement.append(float(np.linalg.norm(truth_delta)))
        aligned_vio = quaternion_product(alignment, vq)
        similarity = abs(np.dot(tq, aligned_vio))
        angle_error.append(math.degrees(2 * math.acos(min(1.0, similarity))))
        truth_yaw.append(math.degrees(math.remainder(yaw(tq) - initial_yaw, math.tau)))
        vio_yaw.append(math.degrees(math.remainder(yaw(aligned_vio) - initial_yaw, math.tau)))
        yaw_error.append(abs(math.remainder(vio_yaw[-1] - truth_yaw[-1], 360.0)))
    return {
        'gazebo_truth_matches': len(matched),
        'gazebo_max_displacement_m': max(truth_displacement),
        'gazebo_position_error_p95_m': p95(translation_error),
        'gazebo_position_error_max_m': max(translation_error),
        'gazebo_attitude_error_p95_deg': p95(angle_error),
        'gazebo_attitude_error_max_deg': max(angle_error),
        'yaw_excursion_gazebo_deg': [min(truth_yaw), max(truth_yaw)],
        'yaw_excursion_vio_deg': [min(vio_yaw), max(vio_yaw)],
        'yaw_change_error_p95_deg': p95(yaw_error),
        'yaw_change_error_max_deg': max(yaw_error),
        'yaw_direction_disagreements': sum(
            abs(reference) >= 10.0 and reference * estimate <= 0
            for reference, estimate in zip(truth_yaw, vio_yaw)),
    }


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--seconds', type=float, default=30.0)
    parser.add_argument('--compare-truth', action='store_true',
                        help='Compare with the evaluation-only Gazebo pose publisher')
    parser.add_argument('--require-yaw', action='store_true',
                        help='Require deliberate yaw in both directions and check it against Gazebo')
    args = parser.parse_args()
    args.compare_truth |= args.require_yaw
    if args.seconds < 5:
        parser.error('--seconds must be at least 5')

    rclpy.init()
    node = rclpy.create_node('phase3_odometry_check')
    clock_ns = None
    captures = set()
    transforms = set()
    states = {}
    samples = []
    truth = {}

    def clock_callback(msg):
        nonlocal clock_ns
        clock_ns = stamp_ns(msg.clock)

    def tf_callback(msg):
        for transform in msg.transforms:
            if (transform.header.frame_id, transform.child_frame_id) == ('odom', 'base_link'):
                transforms.add(stamp_ns(transform.header.stamp))

    def odometry_callback(msg):
        if clock_ns is not None:
            samples.append((msg, clock_ns - stamp_ns(msg.header.stamp), time.monotonic()))

    def truth_callback(msg):
        data = json.loads(msg.data)
        stamp = data['header']['stamp']
        stamp = int(stamp.get('sec', 0)) * 1_000_000_000 + int(stamp.get('nsec', 0))
        for pose in data['pose']:
            if pose.get('name') == 'x500_0':
                p = np.array([pose.get('position', {}).get(axis, 0.0) for axis in 'xyz'])
                q = np.array([pose.get('orientation', {}).get(axis, 0.0) for axis in 'xyzw'])
                q_norm = np.linalg.norm(q)
                if q_norm > 0:
                    truth[stamp] = (p, q / q_norm)
                break

    node.create_subscription(Clock, '/clock', clock_callback, qos_profile_sensor_data)
    node.create_subscription(CameraInfo, '/stereo/left/camera_info',
                             lambda msg: captures.add(stamp_ns(msg.header.stamp)),
                             qos_profile_sensor_data)
    node.create_subscription(TFMessage, '/tf', tf_callback, qos_profile_sensor_data)
    node.create_subscription(VisualSlamStatus, '/visual_slam/status',
                             lambda msg: states.update({stamp_ns(msg.header.stamp): msg.vo_state}),
                             qos_profile_sensor_data)
    node.create_subscription(Odometry, '/visual_slam/tracking/odometry',
                             odometry_callback, qos_profile_sensor_data)
    if args.compare_truth:
        node.create_subscription(String, '/evaluation/gazebo_poses', truth_callback,
                                 qos_profile_sensor_data)

    try:
        start = time.monotonic()
        while time.monotonic() < start + args.seconds:
            rclpy.spin_once(node, timeout_sec=0.05)
    finally:
        node.destroy_node()
        rclpy.shutdown()

    stamps = [stamp_ns(msg.header.stamp) for msg, _, _ in samples]
    if len(stamps) < 2:
        print(json.dumps({'passed': False, 'odometry_samples': len(stamps),
                          'camera_info_samples': len(captures), 'tracking_states': Counter(states.values()),
                          'errors': ['Missing advancing odometry samples']}))
        return 1

    receipt_gaps = [b[2] - a[2] for a, b in zip(samples, samples[1:])]
    ages_ms = [age_ns / 1e6 for _, age_ns, _ in samples]
    positions = [xyz(msg.pose.pose.position) for msg, _, _ in samples]
    orientations = [msg.pose.pose.orientation for msg, _, _ in samples]
    frames = {(msg.header.frame_id, msg.child_frame_id) for msg, _, _ in samples}
    stamp_set = set(stamps)
    tracking_states = Counter(state for stamp, state in states.items() if stamp >= stamps[0])
    errors = []
    if frames != {('odom', 'base_link')} or any(b <= a for a, b in zip(stamps, stamps[1:])):
        errors.append('Unexpected frames or nonadvancing acquisition stamps')
    if not all(math.isfinite(v) for position in positions for v in position):
        errors.append('Nonfinite position')
    if not all(abs(math.hypot(q.x, q.y, q.z, q.w) - 1.0) < 1e-3 for q in orientations):
        errors.append('Invalid orientation quaternion')
    if len(stamp_set & captures) < 0.95 * len(stamps):
        errors.append('Acquisition stamps do not match the stereo camera')
    if len(stamp_set & transforms) < 0.95 * len(stamps):
        errors.append('Missing capture-time odom to base_link transforms')
    if tracking_states[1] < 0.95 * max(len(stamps), tracking_states.total()):
        errors.append('Fewer than 95 percent healthy tracking reports')
    if p95(ages_ms) > 250 or min(ages_ms) < -20 or max(receipt_gaps) > 0.5:
        errors.append('Stale or inconsistent odometry timing during the bring-up sample')
    comparison = truth_errors(truth, stamps, positions, orientations) if args.compare_truth else {}
    if args.compare_truth and comparison['gazebo_truth_matches'] < 0.95 * len(stamps):
        errors.append('Insufficient capture-time Gazebo truth coverage')
    if args.require_yaw:
        low, high = comparison.get('yaw_excursion_gazebo_deg', (0.0, 0.0))
        if low > -15.0 or high < 15.0:
            errors.append('Gazebo truth did not observe at least 15 degrees of yaw each way')
        if (comparison.get('yaw_change_error_p95_deg', math.inf) > 5.0
                or comparison.get('yaw_direction_disagreements', 1) != 0):
            errors.append('VIO yaw error or direction disagreement exceeded the test limits')

    report = {
        'passed': not errors,
        'errors': errors,
        'odometry_samples': len(stamps),
        'odometry_frames': sorted(frames),
        'capture_stamp_matches': len(stamp_set & captures),
        'odometry_tf_matches': len(stamp_set & transforms),
        'tracking_states': tracking_states,
        'receipt_rate_hz': (len(samples) - 1) / (samples[-1][2] - samples[0][2]),
        'capture_rate_hz': (len(stamps) - 1) * 1e9 / (stamps[-1] - stamps[0])
        if stamps[-1] > stamps[0] else None,
        'capture_age_p95_ms': p95(ages_ms),
        'capture_age_max_ms': max(ages_ms),
        'receipt_gap_max_s': max(receipt_gaps),
        'max_displacement_from_first_sample_m': max(math.dist(positions[0], p) for p in positions),
        **comparison,
    }
    print(json.dumps(report, indent=2))
    return bool(errors)


if __name__ == '__main__':
    raise SystemExit(main())
