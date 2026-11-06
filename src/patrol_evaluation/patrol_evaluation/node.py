"""Start barrier, mission logging, markers and sampled evaluation."""
import csv
from dataclasses import asdict
import hashlib
import json
from pathlib import Path
import platform
import re
import subprocess

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy
from std_msgs.msg import Float64, Bool
from geometry_msgs.msg import Point, TransformStamped
from nav_msgs.msg import Odometry
from visualization_msgs.msg import Marker, MarkerArray
from ros_gz_interfaces.msg import Contacts
from controller_manager_msgs.srv import ListControllers
from swarm_interfaces.msg import PhaseState, ControlStatus
from lissajous_path.core import Lissajous, wrap
from lissajous_path.ros_config import scenario_from_node
from .metrics import Sensing


class Evaluation(Node):
    def __init__(self):
        super().__init__('patrol_evaluation');self.c=scenario_from_node(self);self.path=Lissajous(self.c)
        self.physical=self.declare_parameter('physical_ids',[2,3,4]).value
        self.duration=self.declare_parameter('duration',180.0).value
        self.seed=self.declare_parameter('seed',42).value
        count=self.declare_parameter('target_count',10000).value
        speed_min=self.declare_parameter('target_speed_min',0.2).value
        speed_max=self.declare_parameter('target_speed_max',1.0).value
        self.output=Path(self.declare_parameter('output_dir','patrol_runs/latest').value)
        self.output.mkdir(parents=True,exist_ok=True)
        if (self.output/'summary.json').exists() or (self.output/'samples.csv').exists():
            raise RuntimeError(f'Result directory already contains a run: {self.output}; choose a new output_dir')
        self.sensing=Sensing(self.c,count,self.seed,speed_min,speed_max)
        self.states={};self.odom={};self.controls={};self.contact_count=0;self.epoch=None;self.last=None
        self.done=False;self.rows=0;self.minimum=float('inf');self.errors=[];self.solves=[]
        self.controller_clients={i:self.create_client(ListControllers,f'/robot_{i}/controller_manager/list_controllers') for i in self.physical}
        self.controller_futures={};self.controller_ready=set()
        self.failures=0;self.formation=[]
        self.file=(self.output/'samples.csv').open('w',newline='');self.csv=csv.writer(self.file)
        self.csv.writerow(['time','agent','physical','x','y','phase','phase_rate','lateral_error',
                           'min_distance','formation_error','detected','coverage'])
        self.control_file=(self.output/'control.csv').open('w',newline='');self.control_csv=csv.writer(self.control_file)
        self.control_csv.writerow(['time','agent','success','solve_time_ms','lateral_error','steering','throttle','requested_speed','measured_speed','reason'])
        qos=QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.start=self.create_publisher(Float64,'/patrol/start',qos)
        self.finished=self.create_publisher(Bool,'/patrol/finished',qos)
        self.markers=self.create_publisher(MarkerArray,'/patrol/markers',qos)
        for i in self.c.order:
            self.create_subscription(PhaseState,f'/robot_{i}/phase',lambda m,i=i:self.states.update({i:m}),10)
            if i in self.physical:
                self.create_subscription(Odometry,f'/robot_{i}/ground_truth',lambda m,i=i:self.odom.update({i:m}),10)
                self.create_subscription(ControlStatus,f'/robot_{i}/control_status',lambda m,i=i:self.on_control(i,m),10)
                self.create_subscription(Contacts,f'/robot_{i}/contacts',self.on_contact,10)
        metadata={'scenario':asdict(self.c),'physical_ids':list(self.physical),'seed':self.seed,
                  'duration':self.duration,'sensing_radius':self.sensing.radius,
                  'target_count':count,'target_speed':[speed_min,speed_max],
                  'bounds':self.c.bounds(),'max_path_curvature':self.path.max_curvature(),
                  'python':platform.python_version(),'numpy':np.__version__,
                  'source_revision':subprocess.run(['git','rev-parse','HEAD'],capture_output=True,text=True).stdout.strip(),
                  'limitations':['sampled ideal disk sensing','no explicit MPCC collision constraints',
                                 'virtual agents have no physical collision bodies','zero-throttle adapter differs from paper']}
        for key,default in [('feedback',True),('failure_start',38.4),('failure_end',78.0),
                            ('perturb_time',-1.0),('perturb_amount',0.12),('perturb_agent',1)]:
            metadata[key]=self.declare_parameter(key,default).value
        metadata['ros_packages']={name:subprocess.run(['dpkg-query','-W','-f=${Version}',name],
            capture_output=True,text=True).stdout for name in ('ros-humble-gz-ros2-control',
            'ros-humble-ackermann-steering-controller','ros-humble-controller-manager')}
        source=Path.cwd()/'src'
        metadata['source_hashes']={str(p.relative_to(Path.cwd())):hashlib.sha256(p.read_bytes()).hexdigest()
            for p in source.rglob('*') if p.is_file() and p.suffix in ('.py','.xacro','.yaml','.json','.msg')}
        (self.output/'metadata.json').write_text(json.dumps(metadata,indent=2))
        self.create_timer(0.1,self.tick)

    def now(self):return self.get_clock().now().nanoseconds*1e-9

    def on_contact(self,msg):
        if self.epoch is None or self.now()<self.epoch or self.done:return
        for contact in msg.contacts:
            robots=set(re.findall(r'robot_\d+',contact.collision1.name+' '+contact.collision2.name))
            if len(robots)>1:self.contact_count+=1

    def on_control(self,i,msg):
        self.controls[i]=msg
        if self.epoch is not None and self.now()>=self.epoch and not self.done:
            self.control_csv.writerow([self.now()-self.epoch,i,msg.success,msg.solve_time_ms,
                msg.lateral_error,msg.steering,msg.throttle,msg.requested_speed,msg.measured_speed,msg.reason])
            if msg.solve_time_ms>0:self.solves.append(msg.solve_time_ms)
            if not msg.success:self.failures+=1

    def tick(self):
        now=self.now()
        # Visualization must also work before the start barrier and after completion.
        positions=[]
        for i in range(1,len(self.c.order)+1):
            if i in self.odom:
                p=self.odom[i].pose.pose.position;positions.append([p.x,p.y])
            else:
                phase=self.states[i].phase if i in self.states else self.c.phase(i)
                positions.append(self.path.xy(phase))
        self.publish_markers(positions)
        if self.last is not None and now<self.last:
            self.finish('clock_reset');return
        if self.done:return
        if self.epoch is None:
            for i,client in self.controller_clients.items():
                if i in self.controller_ready:continue
                future=self.controller_futures.get(i)
                if future is not None and future.done():
                    response=future.result()
                    if response is not None and any(c.name=='ackermann_steering_controller' and c.state=='active' for c in response.controller):self.controller_ready.add(i)
                    self.controller_futures.pop(i,None)
                if i not in self.controller_futures and client.service_is_ready():self.controller_futures[i]=client.call_async(ListControllers.Request())
        def fresh(m):
            age=now-(m.header.stamp.sec+m.header.stamp.nanosec*1e-9)
            return 0<=age<=0.2
        ready=(len(self.controller_ready)==len(self.physical)
               and all(i in self.states and self.states[i].valid and fresh(self.states[i]) for i in self.c.order)
               and all(i in self.odom and fresh(self.odom[i]) for i in self.physical)
               and all(i in self.controls and fresh(self.controls[i]) for i in self.physical))
        if not ready:self.last=now;return
        if self.epoch is None:
            self.epoch=now+2.;self.start.publish(Float64(data=self.epoch))
            self.get_logger().info(f'All agents ready; mission starts at sim time {self.epoch:.2f}')
        if now<self.epoch:self.last=now;return
        elapsed=now-self.epoch;dt=max(0.,min(.2,now-(self.last if self.last is not None else now)))
        self.last=now
        xy=[];phases=[];errors=[]
        for i in range(1,len(self.c.order)+1):
            state=self.states[i];phases.append(state.phase)
            if i in self.physical:
                p=self.odom[i].pose.pose.position;point=np.array([p.x,p.y])
                _,error=self.path.project(point,state.phase,0,0);self.errors.append(error)
            else:point=self.path.xy(state.phase);error=0.
            xy.append(point);errors.append(error)
        xy=np.array(xy);phases=np.array(phases)
        distances=np.linalg.norm(xy[:,None,:]-xy[None,:,:],axis=-1);np.fill_diagonal(distances,np.inf)
        minimum=float(distances.min());self.minimum=min(self.minimum,minimum)
        offsets=phases-np.array([self.c.phase(i) for i in range(1,len(phases)+1)])
        common=np.arctan2(np.sin(offsets).mean(),np.cos(offsets).mean())
        formation=float(np.sqrt(np.mean(wrap(offsets-common)**2)));self.formation.append(formation)
        sense=self.sensing.update(xy,elapsed,dt)
        for i in range(1,len(phases)+1):
            self.csv.writerow([elapsed,i,i in self.physical,*xy[i-1],phases[i-1],self.states[i].phase_rate,
                               errors[i-1],minimum,formation,sense['detected'],sense['coverage']])
        self.rows+=1
        if self.rows % 10==0:self.file.flush();self.control_file.flush()
        if elapsed>=self.duration:self.finish('duration_reached')

    def publish_markers(self,xy):
        array=MarkerArray();stamp=self.get_clock().now().to_msg()
        path=Marker();path.header.frame_id='world';path.header.stamp=stamp;path.ns='path';path.id=0
        path.type=Marker.LINE_STRIP;path.action=Marker.ADD;path.pose.orientation.w=1.
        path.scale.x=max(.025,.004*max(self.c.A,self.c.B));path.color.r=.6;path.color.g=.6;path.color.b=.6;path.color.a=1.
        path.points=[Point(x=float(x),y=float(y),z=.02) for x,y in self.path.xy(np.linspace(0,2*np.pi,1201))]
        array.markers.append(path)
        for i,point in enumerate(xy,1):
            m=Marker();m.header=path.header;m.ns='agents';m.id=i;m.type=Marker.SPHERE;m.action=Marker.ADD
            m.pose.position.x=float(point[0]);m.pose.position.y=float(point[1]);m.pose.position.z=.15
            m.pose.orientation.w=1.;m.scale.x=m.scale.y=m.scale.z=max(.25,.016*max(self.c.A,self.c.B))
            m.color.b=1. if i in self.physical else 0.;m.color.r=0. if i in self.physical else 1.;m.color.a=1.
            if i in self.physical:
                # An outline keeps the real URDF visible when zooming in.
                radius=m.scale.x
                m.type=Marker.LINE_STRIP;m.scale.x=.05;m.scale.y=m.scale.z=0.
                m.pose.position.z=.02
                m.points=[Point(x=float(radius*np.cos(a)),y=float(radius*np.sin(a)),z=0.)
                          for a in np.linspace(0,2*np.pi,49)]
            array.markers.append(m)
            label=Marker();label.header=path.header;label.ns='labels';label.id=i
            label.type=Marker.TEXT_VIEW_FACING;label.action=Marker.ADD;label.pose.orientation.w=1.
            label.pose.position.x=float(point[0]);label.pose.position.y=float(point[1])+max(.5,.024*max(self.c.A,self.c.B))
            label.pose.position.z=1.;label.scale.z=max(.3,.05*max(self.c.A,self.c.B))
            label.color.r=label.color.g=label.color.b=label.color.a=1.
            label.text=f'robot_{i}' + (' (virtual)' if i not in self.physical else '')
            array.markers.append(label)
        self.markers.publish(array)

    def finish(self,reason):
        if self.done:return
        self.done=True
        if rclpy.ok():self.finished.publish(Bool(data=True))
        summary=dict(reason=reason,samples=self.rows,min_pair_distance=self.minimum if self.rows else None,
            detected=int(self.sensing.detected.sum()),coverage=float(self.sensing.covered.any(axis=0).mean()),
            coverage_per_agent=self.sensing.covered.mean(axis=1).tolist(),
            revisit_max=self.sensing.revisit_max,robot_contact_samples=self.contact_count,
            tracking_rms=float(np.sqrt(np.mean(np.square(self.errors)))) if self.errors else None,
            tracking_max=max(self.errors) if self.errors else None,
            formation_final=self.formation[-1] if self.formation else None,
            failed_control_updates=self.failures,
            solve_ms=dict(zip(('p50','p95','max'),np.percentile(self.solves,[50,95,100]).tolist())) if self.solves else {})
        np.savez_compressed(self.output/'sensing.npz',detection_times=self.sensing.detection_times,
                            grid=self.sensing.grid,covered=self.sensing.covered)
        (self.output/'summary.json').write_text(json.dumps(summary,indent=2,allow_nan=False))
        self.file.flush();self.control_file.flush()
        self.get_logger().info(f'Results written to {self.output}: {reason}')


def main(args=None):
    rclpy.init(args=args);node=Evaluation()
    try:rclpy.spin(node)
    except KeyboardInterrupt:pass
    finally:
        node.finish('interrupted');node.file.close();node.control_file.close()
        node.destroy_node();rclpy.try_shutdown()
