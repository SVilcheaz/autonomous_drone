#!/usr/bin/env python3
"""Stationary range acceptance with evaluation-only named/stamped Gazebo truth."""
import argparse
from collections import OrderedDict, deque
import hashlib
import json
from pathlib import Path
import time

import numpy as np
import rclpy
from rclpy.qos import qos_profile_sensor_data
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import Bool, String

from evaluation_geometry import pose_delta, pose_from_json, read_boxes, reference_raster


def ns(stamp):
    return stamp.sec * 10**9 + stamp.nanosec


def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--scene', type=int, choices=(1, 2, 4), required=True)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--criteria', type=Path, default=Path(__file__).with_name('range_criteria.json'))
    args = parser.parse_args()
    repo = Path(__file__).resolve().parents[2]
    world = repo / f'phase2/worlds/phase2_validation_{args.scene}m.sdf'
    calibration_path = repo / 'phase2/calibration.json'
    criteria = json.loads(args.criteria.read_text())
    calibration = json.loads(calibration_path.read_text())
    boxes = read_boxes(world)
    args.output.mkdir(parents=True, exist_ok=False)  # Preserve every previous run.
    args.output.joinpath('criteria.json').write_text(args.criteria.read_text())
    rclpy.init()
    node = rclpy.create_node('phase2_stationary_range_evaluator')
    state = dict(clock=None, fresh=None, reference=None, truth_count=0,
                 last_truth_stamp=None, first_truth=None, last_truth=None,
                 info=None, bad_pixels=0, evaluated=0, candidates=0,
                 last_evaluated_stamp=None, last_depth_stamp=None,
                 max_position_change=0., max_rotation_change=0.)
    errors, warnings = set(), set()
    confidence_checks = {}
    truth = deque(maxlen=1500)
    images = {side: OrderedDict() for side in ('left', 'right')}
    depth_stamps, receipts, ages, truth_offsets = [], [], [], []
    regions, samples, full_valid = {}, [], []
    started = time.monotonic()
    sample_start = started + criteria['warmup_wall_s']
    sample_end = sample_start + criteria['sample_duration_wall_s']

    def clock_cb(msg):
        stamp = ns(msg.clock)
        if state['clock'] is not None and stamp < state['clock']:
            errors.add('simulation clock reset during acceptance')
        state['clock'] = stamp

    def truth_cb(msg):
        try:
            data = json.loads(msg.data)
            stamp = data['header']['stamp']
            stamp = int(stamp.get('sec', 0))*10**9 + int(stamp.get('nsec', 0))
            required = {'x500_0'} | {b['name'] for b in boxes}
            selected = {}
            for pose in data['pose']:
                name = pose.get('name')
                if name in required:
                    if name in selected:
                        raise ValueError(f'ambiguous truth model name: {name}')
                    selected[name] = pose_from_json(pose)
            if set(selected) != required:
                raise ValueError('truth lacks required named top-level models')
            if stamp <= 0:
                raise ValueError('truth lacks positive simulation timestamp')
            if state['last_truth_stamp'] is not None and stamp <= state['last_truth_stamp']:
                errors.add('nonadvancing Gazebo truth stamp')
            state['last_truth_stamp'] = stamp
            for box in boxes:
                dp, dr = pose_delta(selected[box['name']], (box['position'], box['rotation']))
                if dp > 1e-5 or dr > 1e-5:
                    raise ValueError(f"live {box['name']} pose disagrees with the selected saved world")
            body = selected['x500_0']
            if state['reference'] is None:
                expected, labels, masks, origin, rotation = reference_raster(body, calibration, boxes, criteria)
                state['reference'] = (body, expected, labels, masks, origin, rotation)
                state['first_truth'] = data
                for name in masks:
                    regions[name] = dict(coverage=[], errors=[])
            dp, dr = pose_delta(body, state['reference'][0])
            state['max_position_change'] = max(state['max_position_change'], dp)
            state['max_rotation_change'] = max(state['max_rotation_change'], dr)
            if dp > criteria['maximum_reference_position_change_m'] or dr > criteria['maximum_reference_rotation_change_rad']:
                errors.add('drone moved beyond stationary reference bounds; this run cannot be accepted')
            truth.append((stamp, body))
            state['truth_count'] += 1
            state['last_truth'] = data
        except (ValueError, KeyError, TypeError) as exc:
            errors.add(f'invalid independent truth: {exc}')

    def image_cb(side, msg):
        cache = images[side]
        cache[ns(msg.header.stamp)] = msg
        while len(cache) > 30:
            cache.popitem(last=False)

    def info_cb(msg):
        state['info'] = msg
        expected_p = [400., 0., 320., 0., 0., 400., 240., 0., 0., 0., 1., 0.]
        if (msg.width, msg.height, msg.header.frame_id) != (640, 480, 'stereo_left_optical_frame') or not np.allclose(msg.p, expected_p, atol=1e-6, rtol=0):
            errors.add('live left CameraInfo differs from saved calibration')

    def depth_cb(msg):
        now = time.monotonic()
        if now < sample_start or now > sample_end:
            return
        stamp = ns(msg.header.stamp)
        if state['last_depth_stamp'] is not None and stamp <= state['last_depth_stamp']:
            errors.add('nonadvancing depth source stamp')
        state['last_depth_stamp'] = stamp
        depth_stamps.append(stamp)
        receipts.append(now)
        if state['clock'] is not None:
            ages.append((state['clock'] - stamp)*1e-9)
        if ((msg.width, msg.height, msg.encoding, msg.header.frame_id) !=
                (640, 480, '32FC1', 'stereo_left_optical_frame') or msg.is_bigendian or
                msg.step < msg.width*4 or len(msg.data) != msg.step*msg.height):
            errors.add('malformed metric-depth layout/frame')
            return
        last = state['last_evaluated_stamp']
        if last is not None and stamp-last < criteria['evaluation_period_sim_s']*1e9:
            return
        state['last_evaluated_stamp'] = stamp
        state['candidates'] += 1
        if not truth or state['reference'] is None:
            return
        nearest, _ = min(truth, key=lambda pair: abs(pair[0]-stamp))
        offset = abs(nearest-stamp)*1e-9
        if offset > criteria['maximum_truth_stamp_difference_s']:
            return
        truth_offsets.append(offset)
        observed = np.ndarray((msg.height, msg.width), dtype='<f4', buffer=msg.data,
                              strides=(msg.step, 4)).copy()
        bad = np.isinf(observed) | (np.isfinite(observed) & (observed <= 0))
        state['bad_pixels'] += int(bad.sum())
        valid = np.isfinite(observed)
        full_valid.append(float(valid.mean()))
        _, expected, labels, masks, _, _ = state['reference']
        for name, mask in masks.items():
            covered = valid & mask
            regions[name]['coverage'].append(float(valid[mask].mean()) if mask.any() else None)
            regions[name]['errors'].append(np.abs(observed[covered]-expected[covered]).astype(np.float32))
        state['evaluated'] += 1
        # Three explainable stereo/depth captures, each with EXACT source stamps.
        if len(samples) < 3 and stamp in images['left'] and stamp in images['right']:
            if not samples or now-samples[-1]['receipt_wall'] >= 7:
                name = f'sample_{len(samples):02d}.npz'
                arrays = dict(depth_m=observed, expected_optical_z_m=expected.astype(np.float32),
                              surface_labels=labels, **{f'mask_{k}': v for k, v in masks.items()})
                image_meta = {}
                for side in images:
                    image = images[side][stamp]
                    arrays[f'{side}_bytes'] = np.array(image.data, dtype=np.uint8)
                    image_meta[side] = dict(width=image.width, height=image.height, step=image.step,
                                            encoding=image.encoding, frame=image.header.frame_id)
                np.savez_compressed(args.output/name, **arrays)
                samples.append(dict(path=name, capture_stamp_ns=stamp, receipt_wall=now,
                                    image_metadata=image_meta, truth_stamp_ns=nearest))

    subscriptions = [
        node.create_subscription(Clock, '/clock', clock_cb, qos_profile_sensor_data),
        node.create_subscription(String, '/evaluation/gazebo_poses', truth_cb, qos_profile_sensor_data),
        node.create_subscription(CameraInfo, '/stereo/left/camera_info', info_cb, qos_profile_sensor_data),
        node.create_subscription(Image, '/stereo/left/image_rect', lambda m: image_cb('left', m), qos_profile_sensor_data),
        node.create_subscription(Image, '/stereo/right/image_rect', lambda m: image_cb('right', m), qos_profile_sensor_data),
        node.create_subscription(Image, '/perception/depth', depth_cb, qos_profile_sensor_data),
        node.create_subscription(Bool, '/perception/depth_fresh', lambda m: state.update(fresh=m.data), 10)]
    del subscriptions
    try:
        while time.monotonic() < sample_end:
            rclpy.spin_once(node, timeout_sec=.02)
        if state['evaluated'] < criteria['minimum_evaluated_frames']:
            errors.add('too few evaluated depth frames')
        match_fraction = state['evaluated']/max(1, state['candidates'])
        if match_fraction < criteria['minimum_truth_match_fraction']:
            errors.add('insufficient depth/truth timestamp matches')
        if state['bad_pixels']:
            errors.add('invalid depth represented as zero/negative/infinity instead of NaN')
        if not receipts or time.monotonic()-receipts[-1] > .5 or state['fresh'] is not True:
            errors.add('depth is not fresh at end of run')
        if not ages or min(ages) < -.01 or max(ages) > criteria['maximum_depth_age_sim_s']:
            errors.add('depth capture age violates simulation-clock bound')
        if not state['info']:
            errors.add('no live CameraInfo')
        if len(samples) < 3:
            errors.add('fewer than three exact-stamped source/depth evidence samples')
        metrics = {}
        if state['reference']:
            for name, mask in state['reference'][3].items():
                record = regions[name]
                values = np.concatenate(record['errors']) if record['errors'] else np.array([])
                fractions = [x for x in record['coverage'] if x is not None]
                metrics[name] = dict(mask_pixels=int(mask.sum()),
                    mean_valid_fraction=float(np.mean(fractions)) if fractions else None,
                    minimum_frame_valid_fraction=min(fractions) if fractions else None,
                    median_absolute_error_m=float(np.median(values)) if values.size else None,
                    p95_absolute_error_m=float(np.percentile(values, 95)) if values.size else None)
            roi = metrics['textured_roi']
            if roi['p95_absolute_error_m'] is None or roi['p95_absolute_error_m'] > criteria['maximum_textured_p95_absolute_error_m'][str(args.scene)]:
                errors.add('textured ROI p95 absolute depth error exceeds bound')
            if roi['minimum_frame_valid_fraction'] is None or roi['minimum_frame_valid_fraction'] < criteria['minimum_textured_valid_fraction_each_frame']:
                errors.add('textured ROI valid coverage below bound')
            for name, bound in [('low_texture', 'maximum_low_texture_valid_fraction'),
                                ('right_occluded', 'maximum_right_occluded_valid_fraction')]:
                metric = metrics[name]
                if metric['mask_pixels'] < criteria['minimum_diagnostic_mask_pixels']:
                    if criteria.get('insufficient_diagnostic_region_policy', '').startswith('not_observed;'):
                        confidence_checks[name] = 'not_observed'
                        warnings.add(f'{name}: insufficient visible pixels; combined acceptance requires observation in another scene')
                    else:
                        confidence_checks[name] = 'failed'
                        errors.add(f'{name} mask has too few pixels for confidence acceptance')
                elif metric['mean_valid_fraction'] is None or metric['mean_valid_fraction'] > criteria[bound]:
                    confidence_checks[name] = 'failed'
                    errors.add(f'{name} confidence gate fails: too many pixels reported valid')
                else:
                    confidence_checks[name] = 'passed'
        body = state['reference'][0] if state['reference'] else None
        pipeline_files = [repo / 'src/autonomous_drone_perception' / relative for relative in (
            'src/depth_conversion.cpp', 'src/depth_validity.cpp', 'src/metric_depth.cpp',
            'config/phase2.yaml', 'launch/phase2.launch.py',
            'include/autonomous_drone_perception/depth_conversion.hpp',
            'include/autonomous_drone_perception/depth_validity.hpp')]
        binary = Path('/workspaces/isaac_ros-dev/install/autonomous_drone_perception/lib/autonomous_drone_perception/metric_depth')
        report = dict(passed=not errors, errors=sorted(errors), warnings=sorted(warnings),
            scope='Stationary ground bench only; Gazebo truth is evaluation-only', scene_m=args.scene,
            criteria_sha256=sha(args.criteria), world_sha256=sha(world), calibration_sha256=sha(calibration_path),
            evaluator_sha256=sha(Path(__file__)), geometry_sha256=sha(Path(__file__).with_name('evaluation_geometry.py')),
            pipeline_source_sha256={str(p.relative_to(repo)): sha(p) for p in pipeline_files},
            installed_adapter_sha256=sha(binary),
            depth_frames=len(depth_stamps), evaluated_frames=state['evaluated'], candidates=state['candidates'],
            truth_messages=state['truth_count'], truth_match_fraction=match_fraction,
            max_depth_truth_stamp_difference_s=max(truth_offsets, default=None),
            measured_body_position_world_m=body[0].tolist() if body else None,
            measured_body_rotation_world=body[1].tolist() if body else None,
            max_reference_position_change_m=state['max_position_change'],
            max_reference_rotation_change_rad=state['max_rotation_change'],
            left_optical_origin_world_m=state['reference'][4].tolist() if body else None,
            region_metrics=metrics, confidence_checks=confidence_checks,
            mean_full_image_valid_fraction=float(np.mean(full_valid)) if full_valid else None,
            capture_age_sim_s_median_p95_max=[float(np.median(ages)), float(np.percentile(ages, 95)), max(ages)] if ages else None,
            depth_receipt_rate_wall_hz=(len(receipts)-1)/(receipts[-1]-receipts[0]) if len(receipts)>1 else None,
            depth_source_rate_sim_hz=(len(depth_stamps)-1)*1e9/(depth_stamps[-1]-depth_stamps[0]) if len(depth_stamps)>1 else None,
            depth_fresh=state['fresh'], samples=samples,
            assumptions='Saved SDF box dimensions; live named static poses must agree. Cached ray reference is valid only within declared stationary bounds. Optical Z, not Euclidean range.')
        for name in ('first_truth', 'last_truth'):
            args.output.joinpath(name+'.json').write_text(json.dumps(state[name], indent=2)+'\n')
        args.output.joinpath('report.json').write_text(json.dumps(report, indent=2)+'\n')
        print(json.dumps(report, indent=2))
        return int(bool(errors))
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    raise SystemExit(main())
