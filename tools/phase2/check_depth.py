#!/usr/bin/env python3
"""Inspect live metric-depth contract. This is not the full 1/2/4 m acceptance."""
import argparse
from collections import deque
import json
from pathlib import Path
import time

import numpy as np
import rclpy
from rclpy.qos import qos_profile_sensor_data
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import CameraInfo, Image
from stereo_msgs.msg import DisparityImage
from std_msgs.msg import Bool


def ns(stamp):
    return stamp.sec * 10**9 + stamp.nanosec


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--seconds', type=float, default=15)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    rclpy.init()
    node = rclpy.create_node('phase2_depth_contract_check')
    stamps, receipts, ages, roi_medians, coverage = [], [], [], [], []
    info_stamps = deque(maxlen=1000)
    depth_stamps = []
    raw = {}
    state = {'clock_ns': None, 'fresh': None, 'latest': None, 'violations': set()}
    formats = set()

    def clock_cb(msg):
        state['clock_ns'] = ns(msg.clock)

    def info_cb(msg):
        info_stamps.append(ns(msg.header.stamp))

    def raw_cb(msg):
        raw.update({'frame': msg.header.frame_id, 'f': msg.f, 't': msg.t,
                    'min_disparity': msg.min_disparity, 'max_disparity': msg.max_disparity})

    def depth_cb(msg):
        stamp = ns(msg.header.stamp)
        stamps.append(stamp)
        receipts.append(time.monotonic())
        depth_stamps.append(stamp)
        formats.add((msg.width, msg.height, msg.encoding, msg.header.frame_id))
        if state['clock_ns'] is not None:
            ages.append((state['clock_ns'] - stamp) * 1e-9)
        if msg.encoding != '32FC1' or msg.is_bigendian or msg.step < msg.width * 4 or len(msg.data) != msg.step * msg.height:
            state['violations'].add('malformed metric image')
            return
        image = np.ndarray((msg.height, msg.width), dtype='<f4', buffer=msg.data,
                           strides=(msg.step, 4)).copy()
        if np.any(np.isinf(image)) or np.any(np.isfinite(image) & (image <= 0)):
            state['violations'].add('invalid pixels must be NaN, not zero/negative/infinity')
        state['latest'] = image
        roi = image[80:140, 270:360]  # Saved WSL textured-wall diagnostic ROI.
        valid = np.isfinite(roi)
        coverage.append(float(np.mean(valid)))
        if np.any(valid):
            roi_medians.append(float(np.median(roi[valid])))

    subs = [
        node.create_subscription(Clock, '/clock', clock_cb, qos_profile_sensor_data),
        node.create_subscription(CameraInfo, '/stereo/left/camera_info', info_cb, qos_profile_sensor_data),
        node.create_subscription(DisparityImage, '/perception/disparity_raw', raw_cb, qos_profile_sensor_data),
        node.create_subscription(Image, '/perception/depth', depth_cb, qos_profile_sensor_data),
        node.create_subscription(Bool, '/perception/depth_fresh', lambda m: state.update(fresh=m.data), 10),
    ]
    del subs  # The node owns its subscriptions.
    end = time.monotonic() + args.seconds
    try:
        while time.monotonic() < end:
            rclpy.spin_once(node, timeout_sec=.05)
        errors = sorted(state['violations'])
        if len(stamps) < 5:
            errors.append('fewer than five depth frames received')
        if formats != {(640, 480, '32FC1', 'stereo_left_optical_frame')}:
            errors.append('unexpected dimensions, encoding, or frame')
        if any(b <= a for a, b in zip(stamps, stamps[1:])):
            errors.append('nonadvancing depth capture stamps')
        matched = len(set(depth_stamps) & set(info_stamps))
        if matched < max(2, int(len(set(depth_stamps)) * .95)):
            errors.append('depth is not aligned with received left CameraInfo capture stamps')
        if not receipts or time.monotonic() - receipts[-1] > .5 or state['fresh'] is not True:
            errors.append('depth is not currently fresh')
        if not roi_medians:
            errors.append('no valid depth in the saved textured-wall ROI')
        report = {
            'scope': 'Jetson stereo depth bringup only; full range/coverage/flight gates remain pending',
            'passed': not errors, 'errors': errors, 'depth_frames': len(stamps),
            'formats': [list(x) for x in formats], 'raw_disparity_metadata': raw,
            'source_rate_hz': (len(stamps)-1)*1e9/(stamps[-1]-stamps[0]) if len(stamps)>1 and stamps[-1]>stamps[0] else None,
            'receipt_rate_hz': (len(receipts)-1)/(receipts[-1]-receipts[0]) if len(receipts)>1 else None,
            'max_receipt_gap_wall_s': max((b-a for a,b in zip(receipts, receipts[1:])), default=None),
            'capture_age_sim_s_median_p95': [float(np.median(ages)), float(np.percentile(ages,95))] if ages else None,
            'capture_stamps_matched_to_left_info': matched,
            'last_capture_stamp_ns': stamps[-1] if stamps else None,
            'depth_fresh': state['fresh'],
            'diagnostic_roi_xywh': [270,80,90,60],
            'diagnostic_roi_median_depth_m': float(np.median(roi_medians)) if roi_medians else None,
            'diagnostic_roi_mean_valid_fraction': float(np.mean(coverage)) if coverage else None,
            'note': 'ROI observations are diagnostic; this tool does not establish known surface distance or complete Phase 2 acceptance.',
        }
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(json.dumps(report, indent=2)+'\n')
        if state['latest'] is not None:
            np.save(args.output.with_suffix('.npy'), state['latest'])
        print(json.dumps(report, indent=2))
        return int(bool(errors))
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    raise SystemExit(main())
