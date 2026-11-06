#!/usr/bin/env python3
"""Live Gazebo pause/resume/reset and stopped-agent watchdog checks.

Only sends signals to a coordination process descended from this script's own
ROS launch process. All Gazebo control calls use this test's private partition.
"""
import argparse
import json
import math
import os
from pathlib import Path
import signal
import subprocess
import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy
from nav_msgs.msg import Odometry
from rosgraph_msgs.msg import Clock
from std_msgs.msg import Float64
from swarm_interfaces.msg import PhaseState
import yaml


class Observer(Node):
    def __init__(self):
        super().__init__('gazebo_fault_observer')
        self.sim=None; self.epoch=None; self.odom=None; self.phases=[]
        qos=QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(Clock,'/clock',self.clock,10)
        self.create_subscription(Float64,'/patrol/start',lambda m:setattr(self,'epoch',m.data),qos)
        self.create_subscription(Odometry,'/robot_2/ground_truth',lambda m:setattr(self,'odom',m),10)
        self.create_subscription(PhaseState,'/robot_2/phase',lambda m:self.phases.append(m.phase),10)

    def clock(self,msg):self.sim=msg.clock.sec+msg.clock.nanosec*1e-9

    def wait(self,predicate,timeout=30.):
        end=time.monotonic()+timeout
        while time.monotonic()<end:
            rclpy.spin_once(self,timeout_sec=.02)
            if predicate():return
        raise TimeoutError('Live Gazebo condition did not arrive')

    def pump(self,seconds):
        end=time.monotonic()+seconds
        while time.monotonic()<end:rclpy.spin_once(self,timeout_sec=.02)

    def position(self):
        p=self.odom.pose.pose.position
        return (p.x,p.y)


def world_control(request,env):
    completed=subprocess.run(['ign','service','-s','/world/patrol/control',
        '--reqtype','ignition.msgs.WorldControl','--reptype','ignition.msgs.Boolean',
        '--timeout','5000','--req',request],env=env,capture_output=True,text=True,timeout=10)
    if completed.returncode or 'true' not in completed.stdout:
        raise RuntimeError(f'Gazebo world control failed: {completed.stdout} {completed.stderr}')


def descendants(pid):
    try:children=Path(f'/proc/{pid}/task/{pid}/children').read_text().split()
    except OSError:return []
    result=[]
    for child in children:
        child=int(child);result.append(child);result.extend(descendants(child))
    return result


def coordination_pid(launch_pid):
    for pid in descendants(launch_pid):
        try:args=Path(f'/proc/{pid}/cmdline').read_bytes().split(b'\0')
        except OSError:continue
        # A Python console script appears after its interpreter in /proc/cmdline.
        if any(arg.endswith(b'/agent') for arg in args[:2]) and b'__ns:=/robot_2' in args:return pid
    raise RuntimeError('Own robot_2 coordination process was not found')


def run_case(root,output,name,domain):
    directory=output/name;directory.mkdir()
    config=yaml.safe_load((root/'src/multi_robot_bringup/config/feasible.yaml').read_text())
    config.update(duration=90.,failure_start=999.,failure_end=1000.)
    scenario=directory/'scenario.yaml';scenario.write_text(yaml.safe_dump(config))
    env=os.environ.copy();env.update(ROS_DOMAIN_ID=str(domain),IGN_PARTITION=f'patrol_fault_{os.getpid()}_{name}',
        ROS_LOG_DIR=str(directory/'ros_logs'),PYTHONNOUSERSITE='1',OPENBLAS_NUM_THREADS='1',OMP_NUM_THREADS='1')
    # rclpy initializes its domain at init; set the observer to the same domain.
    os.environ['ROS_DOMAIN_ID']=str(domain)
    command=[sys.executable,'/opt/ros/humble/bin/ros2','launch','multi_robot_bringup','patrol.launch.py',
        'fleet:=single','gui:=false',f'scenario:={scenario}',f'output_dir:={directory}/run',
        f'python:={root}/.venv/bin/python']
    result={'checks':{},'measurements':{}};stopped=None
    with (directory/'launch.log').open('w') as log:
        launch=subprocess.Popen(command,cwd=root,env=env,stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
        rclpy.init();observer=Observer()
        try:
            observer.wait(lambda:observer.epoch is not None and observer.sim is not None and
                          observer.sim>=observer.epoch+(25. if name=='watchdog' else 5.) and observer.odom is not None,60)
            if name=='pause_reset':
                world_control('pause: true',env);observer.pump(.3)
                before=observer.sim;position=observer.position();observer.pump(2.)
                advance=observer.sim-before;movement=math.dist(position,observer.position())
                result['measurements'].update(paused_sim_advance=advance,paused_displacement=movement)
                result['checks']['pause_freezes_clock_and_vehicle']=abs(advance)<.02 and movement<.01
                world_control('pause: false',env)
                observer.wait(lambda:observer.sim>=before+3.,10)
                result['checks']['resume_advances_clock']=True
                result['checks']['resume_moves_vehicle']=math.dist(position,observer.position())>.1
                before_reset=observer.sim;world_control('reset: {all: true}',env)
                observer.wait(lambda:observer.sim<before_reset-1.,10)
                summary=directory/'run/summary.json'
                observer.wait(lambda:summary.exists(),10)
                reason=json.loads(summary.read_text())['reason']
                result['measurements']['reset_completion_reason']=reason
                result['checks']['reset_closes_run_as_clock_reset']=reason=='clock_reset'
            else:
                observer.wait(lambda:observer.phases and abs(observer.odom.twist.twist.linear.x)>.1,10)
                phase=observer.phases[-1];position=observer.position();at_stop=observer.sim
                stopped=coordination_pid(launch.pid);os.kill(stopped,signal.SIGSTOP)
                observer.wait(lambda:observer.sim>=at_stop+1.2,10)
                speed=abs(observer.odom.twist.twist.linear.x)
                movement=math.dist(position,observer.position())
                result['measurements'].update(stale_state_speed=speed,stopping_displacement=movement)
                result['checks']['stale_state_stops_real_vehicle']=speed<.03
                result['checks']['watchdog_stop_distance_bounded']=movement<.7
                index=len(observer.phases);os.kill(stopped,signal.SIGCONT);stopped=None
                observer.wait(lambda:len(observer.phases)>=index+10,10)
                minimum=min(observer.phases[index:index+10])
                result['measurements']['phase_drop_after_resume']=max(0.,phase-minimum)
                result['checks']['phase_continuous_after_data_gap']=phase-minimum<.05
                observer.wait(lambda:abs(observer.odom.twist.twist.linear.x)>.1,10)
                result['checks']['vehicle_resumes_after_agent_returns']=True
            result['passed']=all(result['checks'].values())
        except Exception as exc:
            result['passed']=False;result['error']=str(exc)
        finally:
            if stopped is not None:os.kill(stopped,signal.SIGCONT)
            if launch.poll() is None:
                launch.send_signal(signal.SIGINT)
                try:launch.wait(timeout=10)
                except subprocess.TimeoutExpired:
                    os.killpg(launch.pid,signal.SIGTERM);launch.wait(timeout=10)
            observer.destroy_node();rclpy.try_shutdown()
    (directory/'checks.json').write_text(json.dumps(result,indent=2)+'\n')
    print(name,json.dumps(result),flush=True)
    return result


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output',type=Path,required=True)
    parser.add_argument('--case',choices=['all','pause_reset','watchdog'],default='all')
    args=parser.parse_args();root=Path(__file__).resolve().parent.parent
    output=args.output.resolve();output.mkdir(parents=True,exist_ok=False)
    names=['pause_reset','watchdog'] if args.case=='all' else [args.case]
    results={name:run_case(root,output,name,80+index) for index,name in enumerate(names)}
    (output/'summary.json').write_text(json.dumps(results,indent=2)+'\n')
    return 0 if all(r['passed'] for r in results.values()) else 1


if __name__=='__main__':raise SystemExit(main())
