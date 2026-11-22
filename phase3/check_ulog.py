#!/usr/bin/env python3
"""Verify one bounded Phase 3 SITL flight from its PX4 ULog."""

import argparse
import json
from pathlib import Path

import numpy as np
from pyulog import ULog


flight_topics = (
    'vehicle_status', 'vehicle_land_detected', 'vehicle_local_position',
    'vehicle_local_position_groundtruth', 'vehicle_visual_odometry',
    'estimator_status_flags', 'estimator_aid_src_ev_pos', 'estimator_aid_src_ev_hgt',
)


def between(data, start, end):
    return (data['timestamp'] >= start) & (data['timestamp'] <= end)


def dataset(log, name):
    try:
        return log.get_dataset(name).data
    except IndexError as exc:
        raise ValueError(f'ULog is missing {name}') from exc


def parameter_at(log, name, timestamp):
    value = log.initial_parameters.get(name)
    for updated_at, parameter, updated_value in log.changed_parameters:
        if parameter == name and updated_at <= timestamp:
            value = updated_value
    return value


def verify_height(log, local, status, start, end):
    truth = dataset(log, 'vehicle_local_position_groundtruth')
    selected = between(local, start, end)
    timestamp = local['timestamp'][selected]
    landing = between(status, start, end) & (status['nav_state'] == 18)
    if not timestamp.size or not landing.any():
        raise ValueError('ULog lacks the local position or Land mode interval')
    if truth['timestamp'][0] > start or truth['timestamp'][-1] < end:
        raise ValueError('Gazebo truth does not cover the full armed interval')
    estimated_height = np.interp(start, local['timestamp'], local['z']) - local['z'][selected]
    true_height = (np.interp(start, truth['timestamp'], truth['z'])
                   - np.interp(timestamp, truth['timestamp'], truth['z']))
    error = np.abs(estimated_height - true_height)
    during_landing = timestamp >= status['timestamp'][landing][0]
    # The Gazebo reference identifies the final 5 cm around ground contact.
    airborne_descent = during_landing & (true_height > 0.05)
    if not airborne_descent.any():
        raise ValueError('Gazebo truth has no airborne descent segment in Land mode')
    descent_p95 = float(np.percentile(error[airborne_descent], 95))
    near_ground = error[during_landing & ~airborne_descent]
    estimated_peak = float(estimated_height.max())
    true_peak = float(true_height.max())
    resets = np.unique(local['z_reset_counter'][selected])
    return {
        'passed': bool(resets.size == 1 and estimated_peak <= 1.75 and true_peak <= 1.75
                       and descent_p95 <= 0.30 and error[-1] <= 0.30),
        'simulation_height_limit_m': 1.75,
        'airborne_descent_altitude_error_p95_limit_m': 0.30,
        'ground_contact_height_band_m': 0.05,
        'estimated_peak_height_m': estimated_peak,
        'gazebo_peak_height_m': true_peak,
        'airborne_descent_altitude_error_p95_m': descent_p95,
        'landing_altitude_error_p95_m': float(np.percentile(error[during_landing], 95)),
        'near_ground_landing_altitude_error_peak_m': float(near_ground.max())
        if near_ground.size else None,
        'peak_altitude_error_m': float(error.max()),
        'altitude_error_at_disarm_m': float(error[-1]),
        'armed_z_reset_counters': resets.astype(int).tolist(),
        'barometer_ground_effect_deadzone_m': parameter_at(log, 'EKF2_GND_EFF_DZ', start),
    }


def verify_lateral_bound(log, local, start, end):
    truth = dataset(log, 'vehicle_local_position_groundtruth')
    radii = []
    for positions in (local, truth):
        selected = between(positions, start, end)
        offsets = [positions[axis][selected] - np.interp(start, positions['timestamp'], positions[axis])
                   for axis in ('x', 'y')]
        radii.append(float(np.hypot(*offsets).max()))
    return {
        'passed': max(radii) <= 0.75,
        'simulation_lateral_limit_m': 0.75,
        'estimated_peak_lateral_displacement_m': radii[0],
        'gazebo_peak_lateral_displacement_m': radii[1],
    }


def analyze(log, gnss, flight_index=None):
    status = dataset(log, 'vehicle_status')
    armed_indices = np.flatnonzero(status['arming_state'] == 2)
    if not armed_indices.size:
        raise ValueError('ULog has no armed vehicle status')
    flights = np.split(armed_indices, np.flatnonzero(np.diff(armed_indices) > 1) + 1)
    if flight_index is None:
        if len(flights) != 1:
            raise ValueError(f'ULog contains {len(flights)} flights. Select one with --flight-index.')
        flight_index = 1
    if not 1 <= flight_index <= len(flights):
        raise ValueError(f'Flight index must be between 1 and {len(flights)}')
    armed = flights[flight_index - 1]
    start = int(status['timestamp'][armed[0]])
    disarmed = np.flatnonzero((status['arming_state'] == 1) & (status['timestamp'] > start))
    if not disarmed.size or disarmed[0] < armed[-1]:
        raise ValueError('ULog must contain one flight followed by disarm')
    end = int(status['timestamp'][disarmed[0]])
    flags = dataset(log, 'estimator_status_flags')
    during_flight = between(flags, start, end)
    in_air = during_flight & flags['cs_in_air'].astype(bool)
    if not in_air.any():
        raise ValueError('ULog has no airborne estimator flags')
    air_start = int(flags['timestamp'][in_air][0])
    air_end = int(flags['timestamp'][in_air][-1])

    try:
        ev = log.get_dataset('estimator_aid_src_ev_pos').data
    except IndexError:
        ev = None
    if ev is not None:
        ev_air = between(ev, air_start, air_end)
        fused = ev_air & ev['fused'].astype(bool)
        fusions = np.unique(ev['time_last_fuse'][fused])
        ratios = np.column_stack((ev['test_ratio[0]'][ev_air], ev['test_ratio[1]'][ev_air]))
        finite_ratios = ratios[np.isfinite(ratios)]
        measurements = {
            'airborne_ev_position_aid_samples': int(ev_air.sum()),
            'airborne_ev_position_samples_fused': int(fused.sum()),
            'distinct_airborne_fusion_timestamps': int(fusions.size),
            'airborne_vision_innovation_rejections': int(ev['innovation_rejected'][ev_air].sum()),
            'airborne_vision_innovation_test_ratio_max': float(finite_ratios.max())
            if finite_ratios.size else None,
        }
        fusion_evidence = fusions.size >= 2
    else:
        raw = dataset(log, 'vehicle_visual_odometry')
        raw_air = between(raw, air_start, air_end)
        captures = raw['timestamp_sample'][raw_air]
        resets = np.unique(raw['reset_counter'][raw_air])
        warnings = [msg.message for msg in log.logged_messages if air_start <= msg.timestamp <= air_end
                    and 'ev position' in msg.message.lower() and 'fusion' in msg.message.lower()
                    and any(word in msg.message.lower() for word in ('failing', 'stopping'))]
        measurements = {
            'airborne_vision_input_samples': int(raw_air.sum()),
            'airborne_vision_reset_counters': resets.astype(int).tolist(),
            'airborne_vision_fusion_failure_messages': warnings,
        }
        # A log started before EV aiding can omit the per-measurement topic.
        # For the GNSS-enabled leg use sustained EKF2 flags and real inputs.
        fusion_evidence = bool(gnss == 'active' and captures.size >= 2
                               and (captures[1:] > captures[:-1]).all()
                               and resets.size == 1 and not warnings)

    local = dataset(log, 'vehicle_local_position')
    local_air = between(local, air_start, air_end)
    validity = (local['xy_valid'] & local['z_valid'] & local['v_xy_valid'] & local['v_z_valid'])
    valid = bool(local_air.any() and validity[local_air].all())

    landing = dataset(log, 'vehicle_land_detected')
    final_landing = landing['landed'][(landing['timestamp'] >= air_end)
                                      & (landing['timestamp'] <= end)]
    landed = bool(final_landing.size and final_landing[-1])
    ev_active = int(flags['cs_ev_pos'][in_air].sum())
    gnss_active = np.logical_or.reduce([flags[field] for field in (
        'cs_gnss_pos', 'cs_gnss_vel', 'cs_gps_hgt', 'cs_gnss_yaw')])
    gps_ctrl = parameter_at(log, 'EKF2_GPS_CTRL', start)
    ev_ctrl = parameter_at(log, 'EKF2_EV_CTRL', start)
    if gnss == 'disabled':
        gnss_expected = gps_ctrl == 0 and not gnss_active[during_flight].any()
    else:
        gnss_expected = gps_ctrl not in (None, 0) and bool(flags['cs_gnss_pos'][in_air].any())
    offboard = bool((status['nav_state'][armed] == 14).any())
    height = verify_height(log, local, status, start, end) if gnss == 'disabled' else None
    lateral = verify_lateral_bound(log, local, start, end) if gnss == 'disabled' else None
    height_evidence = True
    if ev_ctrl == 3:
        ev_height = dataset(log, 'estimator_aid_src_ev_hgt')
        height_air = between(ev_height, air_start, air_end)
        height_fused = height_air & ev_height['fused'].astype(bool)
        ev_height_flags = int(flags['cs_ev_hgt'][in_air].sum())
        height_evidence = bool(ev_height_flags == in_air.sum()
                               and np.unique(ev_height['time_last_fuse'][height_fused]).size >= 2)
        measurements.update({
            'airborne_ev_height_flag_samples': ev_height_flags,
            'airborne_ev_height_samples_fused': int(height_fused.sum()),
            'airborne_vision_height_innovation_rejections': int(
                ev_height['innovation_rejected'][height_air].sum()),
        })
    fusion_and_modes_passed = bool(ev_ctrl in (1, 3) and gnss_expected and offboard and landed
                                   and valid and ev_active == in_air.sum() and fusion_evidence
                                   and height_evidence)
    return {
        'passed': (fusion_and_modes_passed and (height is None or height['passed'])
                   and (lateral is None or lateral['passed'])),
        'fusion_and_modes_passed': fusion_and_modes_passed,
        'flight_index': flight_index,
        'flights_in_log': len(flights),
        'expected_gnss': gnss,
        'ekf2_ev_ctrl_at_arming': ev_ctrl,
        'ekf2_gps_ctrl_at_arming': gps_ctrl,
        'ekf2_height_reference_at_arming': parameter_at(log, 'EKF2_HGT_REF', start),
        'armed_duration_sim_s': (end - start) * 1e-6,
        'offboard_observed': offboard,
        'landed_before_disarm': landed,
        'local_state_valid_while_airborne': valid,
        'airborne_estimator_flag_samples': int(in_air.sum()),
        'airborne_ev_position_flag_samples': ev_active,
        'armed_gnss_aiding_flag_samples': int(gnss_active[during_flight].sum()),
        'per_measurement_vision_aid_topic_recorded': ev is not None,
        'height_against_gazebo_truth': height,
        'lateral_bound_against_gazebo_truth': lateral,
        **measurements,
    }


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('ulog', type=Path)
    parser.add_argument('--gnss', choices=('active', 'disabled'), required=True)
    parser.add_argument('--flight-index', type=int, help='One-based flight number in a multi-flight ULog')
    parser.add_argument('--output', type=Path)
    args = parser.parse_args()
    try:
        report = analyze(ULog(str(args.ulog), message_name_filter_list=flight_topics),
                         args.gnss, args.flight_index)
    except (KeyError, ValueError) as exc:
        report = {'passed': False, 'error': str(exc)}
    report['file'] = args.ulog.name
    encoded = json.dumps(report, indent=2) + '\n'
    if args.output:
        args.output.write_text(encoded)
    print(encoded, end='')
    return not report['passed']


if __name__ == '__main__':
    raise SystemExit(main())
