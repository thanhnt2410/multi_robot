"""One distributed ring agent, either measured car or virtual point."""
import math

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy
from nav_msgs.msg import Odometry
from std_msgs.msg import Float64, Bool
from swarm_interfaces.msg import PhaseState, PhaseReference
from lissajous_path.core import Lissajous, phase_rate
from lissajous_path.ros_config import scenario_from_node


class Agent(Node):
    def __init__(self):
        super().__init__('coordination')
        self.c=scenario_from_node(self); self.path=Lissajous(self.c)
        self.id=self.declare_parameter('agent_id',2).value
        self.virtual=self.declare_parameter('virtual',False).value
        self.feedback=self.declare_parameter('feedback',True).value
        self.failure_start=self.declare_parameter('failure_start',38.4).value
        self.failure_end=self.declare_parameter('failure_end',78.0).value
        self.perturb_time=self.declare_parameter('perturb_time',-1.0).value
        self.perturb_amount=self.declare_parameter('perturb_amount',0.12).value
        self.perturb_agent=self.declare_parameter('perturb_agent',1).value
        self.theta=self.c.phase(self.id); self.speed=0.; self.valid=self.virtual
        self.neighbors={}; self.last_odom=None; self.epoch=None; self.perturbed=False
        self.last=None; self.last_ref=None;self.finished=False
        self.pub=self.create_publisher(PhaseState,'phase',10)
        self.ref=self.create_publisher(PhaseReference,'phase_reference',10)
        self.subs=[self.create_subscription(PhaseState,f'/robot_{i}/phase',
                    lambda msg,i=i:self.neighbors.update({i:msg}),10)
                   for i in self.c.neighbors(self.id)]
        if not self.virtual:self.create_subscription(Odometry,'ground_truth',self.odometry,10)
        qos=QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(Float64,'/patrol/start',lambda m:setattr(self,'epoch',m.data),qos)
        self.create_subscription(Bool,'/patrol/finished',lambda m:setattr(self,'finished',m.data),qos)
        self.create_timer(0.02,self.tick)

    def now(self):return self.get_clock().now().nanoseconds*1e-9

    def odometry(self,msg):
        now=self.now(); p=msg.pose.pose.position;q=msg.pose.pose.orientation
        yaw=math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))
        # Gazebo publishes body-frame twist, matching nav_msgs/Odometry.
        self.speed=msg.twist.twist.linear.x
        dt=0 if self.last_odom is None else now-self.last_odom
        if dt<0:
            self.theta=self.c.phase(self.id);dt=0
        elif dt>0.2:
            # A communication gap does not put the vehicle back at its initial
            # position. Retain its last branch/progress and skip extrapolation.
            dt=0
        self.theta,_=self.path.project([p.x,p.y],self.theta,self.speed,dt)
        self.valid=True;self.last_odom=now

    def tick(self):
        now=self.now()
        dt=0 if self.last is None else now-self.last
        if dt<0:
            self.theta=self.c.phase(self.id);self.neighbors.clear();self.epoch=None
            self.perturbed=False;self.last_odom=None;self.valid=self.virtual;dt=0
        self.last=now
        if dt>0.2:dt=0
        ready=self.valid and (self.virtual or self.last_odom is not None and now-self.last_odom<=0.2)
        fresh=lambda m:m.valid and 0<=now-(m.header.stamp.sec+m.header.stamp.nanosec*1e-9)<=0.2
        network=all(i in self.neighbors and fresh(self.neighbors[i]) for i in self.c.neighbors(self.id))
        elapsed=-1 if self.epoch is None else now-self.epoch
        rate=0.
        if ready and network and elapsed>=0 and not self.finished:
            rate=phase_rate(self.theta,[self.neighbors[i].phase for i in self.c.neighbors(self.id)],self.c.omega,self.feedback)
            if self.id in (1,5) and self.failure_start<=elapsed<self.failure_end:rate=0.
            if self.virtual and self.id==self.perturb_agent and not self.perturbed and self.perturb_time>=0 and elapsed>=self.perturb_time:
                self.theta+=self.perturb_amount;self.perturbed=True
            if self.virtual:self.theta+=dt*rate
        state=PhaseState();state.header.stamp=self.get_clock().now().to_msg()
        state.agent_id=self.id;state.phase=float(self.theta)
        state.phase_rate=float(rate if self.virtual else self.speed/self.path.metric(self.theta))
        state.valid=bool(ready);self.pub.publish(state)
        if self.last_ref is None or now-self.last_ref>=0.099 or now<self.last_ref:
            ref=PhaseReference();ref.header=state.header;ref.phase_rate=float(rate)
            ref.valid=bool(ready and network);self.ref.publish(ref);self.last_ref=now


def main(args=None):
    rclpy.init(args=args);node=Agent()
    try:rclpy.spin(node)
    except KeyboardInterrupt:pass
    finally:node.destroy_node();rclpy.try_shutdown()
