#!/usr/bin/env python3
"""Run a bounded Gazebo experiment and stop only the processes it started.

Source install/setup.bash before calling this script.
"""
import argparse
import json
import os
from pathlib import Path
import signal
import subprocess
import sys
import time


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--fleet',choices=['single','hybrid','five'],default='hybrid')
    parser.add_argument('--scenario',type=Path,default=Path('src/multi_robot_bringup/config/feasible.yaml'))
    parser.add_argument('--output',type=Path,required=True)
    parser.add_argument('--timeout',type=float,default=300.)
    parser.add_argument('--record',action='store_true')
    parser.add_argument('--gui',action='store_true')
    args=parser.parse_args();root=Path(__file__).resolve().parent.parent
    out=args.output.resolve();out.mkdir(parents=True,exist_ok=False)
    env=os.environ.copy();env.setdefault('ROS_DOMAIN_ID','85')
    env.setdefault('IGN_PARTITION',f'patrol_{os.getpid()}');env['ROS_LOG_DIR']=str(out/'ros_logs')
    env.update(PYTHONNOUSERSITE='1',OPENBLAS_NUM_THREADS='1',OMP_NUM_THREADS='1')
    cmd=[str(root/'.venv/bin/python'),'/opt/ros/humble/bin/ros2','launch','multi_robot_bringup','patrol.launch.py',
         f'fleet:={args.fleet}',f'gui:={str(args.gui).lower()}',f'scenario:={args.scenario.resolve()}',
         f'output_dir:={out}',f'record:={str(args.record).lower()}',f'python:={root}/.venv/bin/python']
    summary=None
    with (out/'launch.log').open('w') as log:
        process=subprocess.Popen(cmd,cwd=root,env=env,stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
        started=time.monotonic()
        try:
            while process.poll() is None and time.monotonic()-started<args.timeout:
                if (out/'summary.json').exists():
                    try:summary=json.loads((out/'summary.json').read_text())
                    except json.JSONDecodeError:pass
                    if summary is not None:break
                time.sleep(.5)
        finally:
            if process.poll() is None:
                # Let ROS launch forward SIGINT once to each child.
                process.send_signal(signal.SIGINT)
                try:process.wait(timeout=10)
                except subprocess.TimeoutExpired:
                    os.killpg(process.pid,signal.SIGTERM)
                    try:process.wait(timeout=5)
                    except subprocess.TimeoutExpired:os.killpg(process.pid,signal.SIGKILL);process.wait()
    if summary is None:
        print(f'Experiment did not complete; inspect {out}/launch.log',file=sys.stderr);return 1
    print(json.dumps(summary,indent=2))
    return 0 if summary['reason']=='duration_reached' else 1


if __name__=='__main__':raise SystemExit(main())
