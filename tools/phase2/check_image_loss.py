#!/usr/bin/env python3
"""Test-only stereo relay injects image loss before real SGM; preserves stamps.

Launch perception with stereo_namespace:=/evaluation/gated_stereo first. This
tool forwards the real WSL images and CameraInfo, pauses only image forwarding,
then resumes. Clock/IMU and the original WSL sources keep running throughout.
It never publishes operational depth, clock, or IMU, and never replays images.
"""
import argparse
from collections import deque
import hashlib
import json
from pathlib import Path
import time

import rclpy
from rclpy.qos import qos_profile_sensor_data
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import CameraInfo, Image, Imu
from std_msgs.msg import Bool


def ns(stamp):
    return stamp.sec*10**9 + stamp.nanosec


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    if args.output.exists():
        parser.error('output already exists; preserve previous evidence')
    rclpy.init()
    node = rclpy.create_node('phase2_image_loss_injection_evaluation_only')
    state = dict(phase='baseline', clock=None, imu=None, fresh=None,
                 pause=None, resume=None, first_false=None, last_depth=None,
                 baseline_ready=None, pause_clock=None, resume_clock=None,
                 pause_imu=None, resume_imu=None, imu_during_pause=0,
                 resume_recovered=None)
    forwarded = {side: deque(maxlen=2000) for side in ('left', 'right')}
    depths, fresh_events, errors = [], [], set()
    subscriptions, publishers = [], []

    def relay(side, kind, publisher, message):
        if kind == 'image_rect':
            if state['phase'] == 'pause':
                return
            if state['phase'] == 'resume' and ns(message.header.stamp) <= state['resume_clock']:
                return  # Drain any queued captures from before the resume boundary.
            forwarded[side].append(ns(message.header.stamp))
        publisher.publish(message)

    for side in forwarded:
        for kind, msg_type in [('image_rect', Image), ('camera_info', CameraInfo)]:
            pub = node.create_publisher(msg_type, f'/evaluation/gated_stereo/{side}/{kind}', qos_profile_sensor_data)
            publishers.append(pub)
            subscriptions.append(node.create_subscription(msg_type, f'/stereo/{side}/{kind}',
                lambda msg, s=side, k=kind, p=pub: relay(s, k, p, msg), qos_profile_sensor_data))

    def depth_cb(msg):
        now, stamp = time.monotonic(), ns(msg.header.stamp)
        if state['last_depth'] is not None and stamp <= state['last_depth']:
            errors.add('nonadvancing depth capture stamp')
        state['last_depth'] = stamp
        if any(stamp not in forwarded[side] for side in forwarded):
            errors.add('depth stamp was not forwarded in both original source images')
        depths.append(dict(receipt=now, stamp=stamp, phase=state['phase']))
        if state['phase'] == 'pause' and now-state['pause'] > .8:
            errors.add('depth continues after image-loss drain allowance')
        if state['phase'] == 'resume' and state['resume_clock'] is not None and stamp <= state['resume_clock']:
            errors.add('pre-resume depth capture was replayed')

    def fresh_cb(msg):
        now = time.monotonic()
        state['fresh'] = msg.data
        fresh_events.append(dict(receipt=now, data=msg.data, phase=state['phase']))
        if state['phase'] == 'pause' and not msg.data and state['first_false'] is None:
            state['first_false'] = now

    def imu_cb(msg):
        state['imu'] = ns(msg.header.stamp)
        if state['phase'] == 'pause':
            state['imu_during_pause'] += 1

    subscriptions += [
        node.create_subscription(Clock, '/clock', lambda m: state.update(clock=ns(m.clock)), qos_profile_sensor_data),
        node.create_subscription(Imu, '/imu/data', imu_cb, qos_profile_sensor_data),
        node.create_subscription(Image, '/perception/depth', depth_cb, qos_profile_sensor_data),
        node.create_subscription(Bool, '/perception/depth_fresh', fresh_cb, 10)]
    started = time.monotonic()
    try:
        while time.monotonic()-started < 40:
            rclpy.spin_once(node, timeout_sec=.01)
            now = time.monotonic()
            if state['phase'] == 'baseline':
                if state['fresh'] is True and len(depths) >= 10 and state['clock'] and state['imu']:
                    if state['baseline_ready'] is None:
                        state['baseline_ready'] = now
                    if now-state['baseline_ready'] >= 2:
                        state.update(phase='pause', pause=now, pause_clock=state['clock'], pause_imu=state['imu'])
                        print('Pausing only stereo image forwarding for 3 seconds', flush=True)
            elif state['phase'] == 'pause' and now-state['pause'] >= 3:
                if state['fresh'] is not False:
                    errors.add('freshness did not remain false during image loss')
                if state['clock']-state['pause_clock'] < 10**9 or state['imu'] <= state['pause_imu'] or state['imu_during_pause'] < 100:
                    errors.add('clock/IMU did not continue through the image-only pause')
                if state['first_false'] is None or state['first_false']-state['pause'] > .8:
                    errors.add('freshness did not expire within 0.8 s of image pause')
                state.update(phase='resume', resume=now, resume_clock=state['clock'], resume_imu=state['imu'])
                print('Resuming original stereo forwarding', flush=True)
            elif state['phase'] == 'resume':
                recovered = [d for d in depths if d['phase'] == 'resume']
                if len(recovered) >= 5 and state['fresh'] is True and state['resume_recovered'] is None:
                    state['resume_recovered'] = now
                if now-state['resume'] >= 3:
                    if state['resume_recovered'] is None or state['resume_recovered']-state['resume'] > 2:
                        errors.add('fresh advancing depth did not recover within 2 s')
                    break
        else:
            errors.add('timed out waiting for live pipeline baseline or recovery')
        if state['pause'] is None:
            errors.add('image-loss test never reached a healthy baseline')
        pause_depths = [d for d in depths if d['phase'] == 'pause']
        last_pause_depth = max((d['receipt'] for d in pause_depths), default=state['pause'])
        report = dict(passed=not errors, errors=sorted(errors),
            scope='ROS image supply loss before real SGM via evaluation-only relay; original Gazebo source remains running',
            source_headers_preserved=True, originals='/stereo', gated_inputs='/evaluation/gated_stereo',
            installed_adapter_sha256=hashlib.sha256(Path('/workspaces/isaac_ros-dev/install/autonomous_drone_perception/lib/autonomous_drone_perception/metric_depth').read_bytes()).hexdigest(),
            pause_wall_s=3, drain_allowance_wall_s=.8, configured_adapter_timeout_wall_s=.5,
            false_after_pause_wall_s=state['first_false']-state['pause'] if state['first_false'] else None,
            false_after_last_drain_depth_wall_s=state['first_false']-last_pause_depth if state['first_false'] else None,
            clock_advance_during_pause_sim_s=(state['resume_clock']-state['pause_clock'])*1e-9 if state['resume_clock'] else None,
            imu_frames_during_pause=state['imu_during_pause'],
            recovered_after_resume_wall_s=state['resume_recovered']-state['resume'] if state['resume_recovered'] else None,
            depth_frames=len(depths), forwarded_frames={s:len(v) for s,v in forwarded.items()},
            transition_state=state, depth_events=depths, freshness_events=fresh_events,
            limitations='Tests loss of images supplied to SGM, not a Gazebo camera shutdown. Perception must be restored to /stereo after this tool exits.')
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(json.dumps(report, indent=2)+'\n')
        print(json.dumps({k:v for k,v in report.items() if not k.endswith('_events')}, indent=2))
        return int(bool(errors))
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    raise SystemExit(main())
