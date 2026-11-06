#!/usr/bin/env python3
"""Sequential Gazebo physics scenarios with event and outcome checks.

Source ROS and install/setup.bash; run with the repository's .venv interpreter.
The scenarios run sequentially so CPU contention does not bias solver deadlines.
"""
import argparse
import csv
import json
import os
from pathlib import Path
import subprocess
import sys
import time

import numpy as np
import yaml
from lissajous_path.core import Lissajous, Scenario


CASES = [
    ('normal', 'hybrid', {'failure_start': 999., 'failure_end': 1000.}),
    ('failure_recovery', 'hybrid', {}),
    ('open_loop_failure', 'hybrid', {'feedback': False}),
    ('phase_perturbation', 'hybrid', {'failure_start': 999., 'failure_end': 1000.,
                                    'perturb_time': 60., 'perturb_amount': .12, 'perturb_agent': 1}),
    ('five_physical', 'five', {}),
]


def analyze(directory, name):
    summary = json.loads((directory/'summary.json').read_text())
    metadata = json.loads((directory/'metadata.json').read_text())
    with (directory/'samples.csv').open() as f:
        rows = list(csv.DictReader(f))
    with (directory/'control.csv').open() as f:
        controls = list(csv.DictReader(f))
    path = Lissajous(Scenario(**metadata['scenario']))
    times = np.array([float(r['time']) for r in rows])
    checks = {
        'completed_180_sim_seconds': summary['reason'] == 'duration_reached' and times.max() >= 180.,
        'no_inter_robot_contact': summary['robot_contact_samples'] == 0,
        'separation_above_footprint_diameter': summary['min_pair_distance'] > .84,
        'tracking_rms_below_15cm': summary['tracking_rms'] < .15,
        'tracking_max_below_50cm': summary['tracking_max'] < .5,
        'failed_updates_below_5_percent': summary['failed_control_updates']/max(1, len(controls)) < .05,
        'ideal_team_coverage_complete': summary['coverage'] == 1.,
        'ideal_targets_all_detected': summary['detected'] == metadata['target_count'],
    }
    events = {}
    if name in ('normal', 'failure_recovery', 'phase_perturbation', 'five_physical'):
        checks['formation_recovered_below_0_03_rad'] = summary['formation_final'] < .03
    if name in ('failure_recovery', 'open_loop_failure', 'five_physical'):
        for agent in (1, 5):
            subset = [r for r in rows if int(r['agent']) == agent and 60. <= float(r['time']) <= 75.]
            rates = np.abs([float(r['phase_rate']) for r in subset])
            events[f'stopped_agent_{agent}_max_phase_rate'] = float(rates.max()) if len(rates) else None
            checks[f'agent_{agent}_stopped_during_failure'] = len(rates) > 0 and rates.max() < .002
            after = [r for r in rows if int(r['agent']) == agent and float(r['time']) >= 120.]
            progress = float(after[-1]['phase'])-float(after[0]['phase']) if after else 0.
            events[f'agent_{agent}_phase_progress_after_recovery'] = progress
            checks[f'agent_{agent}_resumed'] = progress > .1
        if name != 'open_loop_failure':
            subset = [r for r in rows if r['physical'] == 'True' and 60. <= float(r['time']) <= 75.]
            speeds = [abs(float(r['phase_rate'])*path.metric(float(r['phase']))) for r in subset]
            events['mean_physical_speed_late_failure'] = float(np.mean(speeds)) if speeds else None
            checks['physical_team_settled_during_failure'] = bool(speeds) and np.mean(speeds) < .05
    if name == 'phase_perturbation':
        subset = [r for r in rows if int(r['agent']) == 1 and 59.7 <= float(r['time']) <= 60.4]
        phases = np.array([float(r['phase']) for r in subset])
        jump = float(np.max(np.diff(phases))) if len(phases) > 1 else 0.
        events['observed_phase_jump'] = jump
        checks['injected_phase_jump_observed'] = jump > .08
    if name == 'open_loop_failure':
        checks['expected_baseline_formation_degradation'] = summary['formation_final'] > .05
    result = dict(passed=all(checks.values()), checks={k: bool(v) for k,v in checks.items()},
                  event_measurements=events, summary=summary,
                  failed_update_fraction=summary['failed_control_updates']/max(1,len(controls)))
    (directory/'checks.json').write_text(json.dumps(result, indent=2)+'\n')
    return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--timeout-per-case', type=float, default=420.)
    args = parser.parse_args()
    root = Path(__file__).resolve().parent.parent
    output = args.output.resolve(); output.mkdir(parents=True, exist_ok=False)
    (output/'scenarios').mkdir()
    base = yaml.safe_load((root/'src/multi_robot_bringup/config/feasible.yaml').read_text())
    results = {}
    for index, (name, fleet, changes) in enumerate(CASES):
        scenario = {**base, **changes, 'duration': 180.}
        config = output/'scenarios'/f'{name}.yaml'; config.write_text(yaml.safe_dump(scenario))
        env = os.environ.copy(); env['ROS_DOMAIN_ID'] = str(70+index)
        env['IGN_PARTITION'] = f'patrol_suite_{os.getpid()}_{name}'
        command = [sys.executable, str(root/'scripts/run_experiment.py'), '--fleet', fleet,
                   '--scenario', str(config), '--output', str(output/name),
                   '--timeout', str(args.timeout_per_case)]
        print(f'START {name}: {fleet}, 180 simulation seconds', flush=True)
        with (output/f'{name}.runner.log').open('w') as log:
            child = subprocess.Popen(command, cwd=root, env=env, stdout=log, stderr=subprocess.STDOUT)
            last_report = time.monotonic()
            try:
                while child.poll() is None:
                    time.sleep(1.)
                    if time.monotonic()-last_report >= 30.:
                        samples = output/name/'samples.csv'; latest = 'waiting for startup'
                        if samples.exists() and samples.stat().st_size:
                            with samples.open('rb') as f:
                                f.seek(max(0, samples.stat().st_size-4096)); lines=f.read().decode().splitlines()
                            if lines and lines[-1].split(',')[0] != 'time':latest = lines[-1].split(',')[0]+' sim s'
                        print(f'PROGRESS {name}: {latest}', flush=True); last_report=time.monotonic()
            except BaseException:
                child.send_signal(__import__('signal').SIGINT)
                child.wait(timeout=30)
                raise
        try:
            results[name] = analyze(output/name, name)
        except (OSError, ValueError, KeyError) as exc:
            results[name] = {'passed': False, 'error': str(exc)}
        results[name]['runner_exit_code'] = child.returncode
        results[name]['passed'] = results[name]['passed'] and child.returncode == 0
        print(f'END {name}: {"PASS" if results[name]["passed"] else "FAIL"}', flush=True)
        (output/'suite_summary.json').write_text(json.dumps(results, indent=2)+'\n')
    feedback = results.get('failure_recovery',{}).get('summary',{}).get('formation_final')
    baseline = results.get('open_loop_failure',{}).get('summary',{}).get('formation_final')
    comparison = dict(feedback_final_formation=feedback, open_loop_final_formation=baseline,
                      feedback_improves_formation=feedback is not None and baseline is not None and feedback < baseline)
    (output/'comparison.json').write_text(json.dumps(comparison, indent=2)+'\n')
    print(json.dumps(comparison,indent=2), flush=True)
    return 0 if all(r['passed'] for r in results.values()) and comparison['feedback_improves_formation'] else 1


if __name__ == '__main__':
    raise SystemExit(main())
