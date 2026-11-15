#!/usr/bin/env python3
"""Combine three range runs and the actual-pipeline image-loss test honestly."""
import argparse
import hashlib
import json
from pathlib import Path


def assess(runs, loss):
    errors = []
    scenes = sorted(r.get('scene_m') for r in runs)
    if scenes != [1, 2, 4]:
        errors.append('exactly one 1 m, 2 m, and 4 m report is required')
    if not all(r.get('passed') is True for r in runs):
        errors.append('at least one individual range run failed')
    for key in ('criteria_sha256', 'calibration_sha256', 'evaluator_sha256',
                'geometry_sha256', 'pipeline_source_sha256', 'installed_adapter_sha256'):
        values = [r.get(key) for r in runs]
        if not values or any(v is None for v in values) or any(v != values[0] for v in values[1:]):
            errors.append(f'inconsistent or missing {key} across range runs')
    for name in ('low_texture', 'right_occluded'):
        statuses = [r.get('confidence_checks', {}).get(name) for r in runs]
        if 'passed' not in statuses or any(s not in ('passed', 'not_observed') for s in statuses):
            errors.append(f'{name} must be observed and passed in at least one scene, with no failed scenes')
    if loss.get('passed') is not True:
        errors.append('actual-pipeline image-loss/recovery check has not passed')
    if not runs or loss.get('installed_adapter_sha256') != runs[0].get('installed_adapter_sha256'):
        errors.append('image-loss check must use the same adapter binary as the range runs')
    return errors


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--runs', type=Path, nargs=3, required=True)
    parser.add_argument('--image-loss', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    if args.output.exists():
        parser.error('output already exists; preserve previous evidence')
    paths = [p/'report.json' if p.is_dir() else p for p in args.runs]
    runs = [json.loads(p.read_text()) for p in paths]
    loss = json.loads(args.image_loss.read_text())
    errors = assess(runs, loss)
    result = dict(passed=not errors, errors=errors,
        summarizer_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
        scope='Chronology step 9 only: stationary simulated stereo range, invalid-data handling and ROS image-supply loss',
        accepted_operating_range_m=[1, 4] if not errors else None,
        tested_distances_m=sorted(r['scene_m'] for r in runs),
        range_runs={str(r['scene_m']):str(p) for r,p in zip(runs,paths)},
        input_sha256={str(p):hashlib.sha256(p.read_bytes()).hexdigest() for p in paths+[args.image_loss]},
        metrics={str(r['scene_m']):dict(regions=r['region_metrics'],confidence_checks=r.get('confidence_checks'),
            depth_receipt_rate_wall_hz=r['depth_receipt_rate_wall_hz'],
            capture_age_sim_s_median_p95_max=r['capture_age_sim_s_median_p95_max']) for r in runs},
        image_loss_report=str(args.image_loss),
        limitations=['Bounds apply to the specified textured wall ROI in the saved ideal stationary scenes.',
                     'Unknown depth is not free space. Uniform surfaces, borders, ground, and obstacle edges have reduced coverage.',
                     'Some erroneous valid edge pixels remain; this is not a guarantee for every image pixel or arbitrary real surfaces.',
                     'Image loss is injected at the ROS supply to SGM; Gazebo cameras are not shut down.',
                     'Moving-camera quality, real sensors, concurrent hover, resource headroom and screenshot evidence remain separate gates.'],
        full_phase2_complete=False)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(result,indent=2)+'\n')
    print(json.dumps(result,indent=2))
    return int(bool(errors))


if __name__ == '__main__':
    raise SystemExit(main())
