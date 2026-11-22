#!/usr/bin/env python3
"""Run and record one bounded Phase 3 route against the WSL Gazebo SITL instance."""

import argparse
import json
import math
from pathlib import Path
import subprocess
import sys
import time

import rclpy
from px4_msgs.msg import (
    EstimatorStatusFlags, OffboardControlMode, VehicleLandDetected, VehicleLocalPosition,
    VehicleOdometry, VehicleStatus, VehicleCommand,
)
from rclpy.qos import DurabilityPolicy, QoSProfile, qos_profile_sensor_data as qos
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import Image
from std_msgs.msg import Bool, String
from std_srvs.srv import Trigger

sys.path.insert(0, str(Path.cwd() / 'tools' / 'phase2'))
from coexistence_metrics import parse_tegrastats, percentile, stream_stats

parser = argparse.ArgumentParser(description=__doc__)
parser.add_argument('--gnss', choices=('active', 'disabled'), required=True)
parser.add_argument('--output', type=Path, required=True)
parser.add_argument('--vision-height', action='store_true')
parser.add_argument('--local-offboard', action='store_true')
args = parser.parse_args()
args.output.mkdir(parents=True, exist_ok=False)
rclpy.init()
node = rclpy.create_node('phase3_sitl_flight_observer')
data = {}
latest = {}
streams = {name: [] for name in ('vision', 'depth', 'heartbeat', 'flags', 'position', 'status', 'landing')}
origins = time.monotonic()
now = lambda: time.monotonic() - origins

def save_sample(name, **payload):
    latest[name] = (now(), payload)
    if name in streams:
        streams[name].append((latest[name][0], payload))

def subscribe(msg_type, path, callback):
    version = getattr(msg_type, 'MESSAGE_VERSION', 0)
    topic = '/fmu/' + path + (f'_v{version}' if version else '')
    node.create_subscription(msg_type, topic, callback, qos)

node.create_subscription(Clock, '/clock',
    lambda m: data.update(clock=m.clock.sec * 1_000_000 + m.clock.nanosec // 1000), qos)
node.create_subscription(String, '/mission/state',
    lambda m: save_sample('mission', state=m.data),
    QoSProfile(depth=5, durability=DurabilityPolicy.TRANSIENT_LOCAL))
node.create_subscription(Bool, '/perception/depth_fresh', lambda m: save_sample('depth_fresh', fresh=m.data), qos)
def on_depth(msg):
    stamp = msg.header.stamp.sec * 1_000_000 + msg.header.stamp.nanosec // 1000
    save_sample('depth', capture_us=stamp, age_us=data.get('clock', stamp) - stamp)
node.create_subscription(Image, '/perception/depth', on_depth, qos)
subscribe(VehicleOdometry, 'in/vehicle_visual_odometry', lambda m: save_sample('vision',
          capture_us=m.timestamp_sample, age_us=data.get('clock', m.timestamp)-m.timestamp_sample, reset=m.reset_counter))
subscribe(EstimatorStatusFlags, 'out/estimator_status_flags', lambda m: save_sample('flags',
          ev=bool(m.cs_ev_pos), ev_height=bool(m.cs_ev_hgt), gnss=bool(m.cs_gnss_pos or m.cs_gnss_vel or m.cs_gps_hgt or m.cs_gnss_yaw),
          in_air=bool(m.cs_in_air)))
subscribe(OffboardControlMode, 'in/offboard_control_mode', lambda _: save_sample('heartbeat'))
subscribe(VehicleLandDetected, 'out/vehicle_land_detected', lambda m: save_sample('landing', landed=bool(m.landed)))
subscribe(VehicleStatus, 'out/vehicle_status', lambda m: save_sample('status',
          armed=m.arming_state == VehicleStatus.ARMING_STATE_ARMED,
          disarmed=m.arming_state == VehicleStatus.ARMING_STATE_DISARMED,
          offboard=m.nav_state == VehicleStatus.NAVIGATION_STATE_OFFBOARD,
          checks=bool(m.pre_flight_checks_pass), failsafe=bool(m.failsafe),
          system_id=m.system_id, component_id=m.component_id))
subscribe(VehicleLocalPosition, 'out/vehicle_local_position', lambda m: save_sample('position',
          valid=bool(m.xy_valid and m.z_valid and m.v_xy_valid and m.v_z_valid
                     and all(math.isfinite(v) for v in (m.x, m.y, m.z, m.heading))),
          x=m.x, y=m.y, z=m.z, xy_global=bool(m.xy_global), z_global=bool(m.z_global)))

def current(name, limit=1.6):
    t, m = latest.get(name, (-float('inf'), {}))
    return m if now() - t <= limit else {}

def wait_for(predicate, duration):
    until = now() + duration
    while now() < until:
        rclpy.spin_once(node, timeout_sec=0.035)
        if predicate():
            return True
    return False

def prepare(require_mode=True):
    return (latest.get('mission', (0, {}))[1].get('state') == 'IDLE'
            and current('landing').get('landed') and current('status').get('disarmed')
            and (not require_mode or current('status').get('checks') or current('status').get('offboard'))
            and not current('status').get('failsafe')
            and current('position').get('valid') and current('flags').get('ev')
            and (not args.local_offboard or (not current('position').get('xy_global')
                                              and not current('position').get('z_global')
                                              and math.hypot(current('position')['x'], current('position')['y']) < 5.0))
            and (not args.vision_height or current('flags').get('ev_height'))
            and current('flags').get('gnss') == (args.gnss == 'active')
            and data.get('clock', 0) > 0
            and current('vision', 0.25).get('age_us', float('inf')) < 250_000
            and (args.gnss == 'active' or (current('depth_fresh').get('fresh')
                                          and current('depth', 0.5))))

def send(client):
    response = client.call_async(Trigger.Request())
    if not wait_for(response.done, 5.0):
        raise RuntimeError('Mission service timed out')
    result = response.result()
    if not result.success:
        raise RuntimeError(result.message)

start_client = node.create_client(Trigger, '/mission/start')
land_client = node.create_client(Trigger, '/mission/land')
started = False
mission_complete = False
failure = None
bootstrap_timer = None
with (args.output / 'tegrastats.log').open('w') as res_log:
    tegra = subprocess.Popen(['tegrastats', '--interval', '500'], stdout=res_log,
                             stderr=subprocess.STDOUT)
    try:
        if args.local_offboard:
            heartbeat = node.create_publisher(OffboardControlMode, '/fmu/in/offboard_control_mode', 10)
            commands = node.create_publisher(VehicleCommand, '/fmu/in/vehicle_command', 10)
            if not wait_for(lambda: prepare(require_mode=False) and heartbeat.get_subscription_count()
                            and commands.get_subscription_count(), 14):
                raise RuntimeError('Prestream requires a disarmed meter-scale local estimate and fresh vision')

            def prestream():
                if current('status').get('disarmed') and current('position').get('valid'):
                    heartbeat.publish(OffboardControlMode(timestamp=data['clock'], position=True))

            bootstrap_timer = node.create_timer(0.05, prestream)
            wait_for(lambda: False, 1.8)
            status_now = current('status')
            if (not prepare(require_mode=False)
                    or (status_now.get('system_id'), status_now.get('component_id')) != (1, 1)):
                raise RuntimeError('SITL readiness changed during Offboard prestream')
            commands.publish(VehicleCommand(
                timestamp=data['clock'], command=VehicleCommand.VEHICLE_CMD_DO_SET_MODE,
                param1=1.0, param2=6.0, target_system=1, target_component=1,
                source_system=254, source_component=192, from_external=True))
            if not wait_for(lambda: current('status').get('offboard')
                            and current('status').get('checks'), 9):
                raise RuntimeError('PX4 did not approve disarmed local Offboard preflight')
            print('PX4 approved preflight in local Offboard mode.', flush=True)

        if not wait_for(lambda: prepare() and start_client.service_is_ready(), 14):
            raise RuntimeError(f'SITL preflight failed with {latest}')
        anchor = current('position')
        print(f'Preflight passed with GNSS {args.gnss}; starting bounded SITL route.', flush=True)
        send(start_client)
        if bootstrap_timer:
            bootstrap_timer.cancel()
        started = True
        wait_for(lambda: any(m['armed'] for _, m in streams['status']), 12)
        abort_cause = None
        deadline = now() + 105
        unhealthy_since = None
        while now() < deadline:
            rclpy.spin_once(node, timeout_sec=0.04)
            state = latest.get('mission', (0, {}))[1].get('state')
            if state in ('COMPLETE', 'FAILED'):
                mission_complete = state == 'COMPLETE'
                if not mission_complete:
                    failure = 'Controller reported FAILED'
                break
            if current('status').get('armed') and current('landing').get('landed') is False:
                flags = current('flags')
                loss = (not flags.get('ev') or flags.get('gnss') != (args.gnss == 'active')
                        or (args.vision_height and not flags.get('ev_height'))
                        or not current('vision', 0.5) or not current('position').get('valid')
                        or (args.gnss == 'disabled' and not current('depth_fresh').get('fresh')))
                if loss:
                    unhealthy_since = now() if unhealthy_since is None else unhealthy_since
                    if now() - unhealthy_since >= 1.5 and abort_cause is None:
                        abort_cause = 'Sustained loss of required VIO, EKF2, or depth state'
                        print(abort_cause, flush=True)
                        if land_client.service_is_ready():
                            send(land_client)
                else:
                    unhealthy_since = None
        else:
            failure = 'Controller did not reach a terminal state'
        if abort_cause:
            failure = abort_cause
    except Exception as exc:
        failure = str(exc)
        print(failure, flush=True)
    finally:
        if bootstrap_timer:
            bootstrap_timer.cancel()
        if started and current('status').get('armed') and land_client.service_is_ready():
            try:
                send(land_client)
                wait_for(lambda: current('status').get('disarmed') and current('landing').get('landed'), 65)
            except Exception as exc:
                failure = f'{failure}; landing request: {exc}'
        wait_for(lambda: current('status').get('disarmed') and current('landing').get('landed'), 4)
        landed_disarmed = bool(current('status').get('disarmed') and current('landing').get('landed'))
        tegra.terminate()
        tegra.wait(timeout=5)
        node.destroy_node()
        rclpy.shutdown()

armed = [t for t, m in streams['status'] if m['armed']]
offboard = [t for t, m in streams['status'] if m['armed'] and m['offboard']]
in_air = [(t, m) for t, m in streams['flags'] if m['in_air']]
samp = {}
if armed:
    for key in ('depth', 'vision', 'heartbeat'):
        window = offboard if key == 'heartbeat' and offboard else armed
        start, end = window[0], window[-1]
        selected = [(t, m) for t, m in streams[key] if start <= t <= end]
        samp[key] = stream_stats([t for t, _ in selected], start, end)
        ages = [m['age_us'] for _, m in selected if 'age_us' in m]
        if ages:
            samp[key]['capture_age_p95_us'] = percentile(ages, 95)
            samp[key]['capture_age_max_us'] = max(ages)
    positions = [m for t, m in streams['position'] if armed[0] <= t <= armed[-1] and m['valid']]
else:
    positions = []

resources = parse_tegrastats((args.output / 'tegrastats.log').read_text())
report = {
    'passed_live': bool(started and mission_complete and landed_disarmed and offboard and in_air and not failure),
    'gnss': args.gnss, 'local_offboard_prestream': args.local_offboard, 'failure': failure, 'started': started, 'mission_complete': mission_complete,
    'landed_disarmed': landed_disarmed, 'airborne_ev_flags': sum(m['ev'] for _, m in in_air),
    'airborne_ev_height_flags': sum(m['ev_height'] for _, m in in_air),
    'airborne_gnss_flags': sum(m['gnss'] for _, m in in_air), 'airborne_flag_samples': len(in_air),
    'peak_lateral_displacement_m': max((math.hypot(m['x']-anchor['x'], m['y']-anchor['y'])
                                        for m in positions), default=None),
    'peak_height_above_takeoff_m': max((anchor['z']-m['z'] for m in positions), default=None),
    'armed_offboard_duration_wall_s': offboard[-1]-offboard[0] if offboard else 0,
    'vision_reset_counters': sorted({m['reset'] for t, m in streams['vision']
                                     if armed and armed[0] <= t <= armed[-1]}),
    'streams': samp, 'tegrastats': resources, 'px4_ulog_verified': False,
}
(args.output / 'jetson_report.json').write_text(json.dumps(report, indent=2) + '\n')
print(json.dumps(report, indent=2))
raise SystemExit(not report['passed_live'])
