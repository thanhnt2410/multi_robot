"""MPCC at 10 Hz, throttle-to-velocity adapter at 100 Hz."""
import math
import os

# Avoid BLAS thread oversubscription when several independent solvers run.
os.environ.setdefault('OPENBLAS_NUM_THREADS','1')
os.environ.setdefault('OMP_NUM_THREADS','1')
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TwistStamped
from swarm_interfaces.msg import PhaseState, PhaseReference, ControlStatus
from lissajous_path.core import Lissajous
from lissajous_path.ros_config import scenario_from_node
from .solver import MPCC, Vehicle


class Controller(Node):
    def __init__(self):
        super().__init__('mpcc')
        self.path=Lissajous(scenario_from_node(self))
        defaults=Vehicle()
        self.vehicle=Vehicle(**{k:self.declare_parameter(k,float(v)).value for k,v in vars(defaults).items()})
        self.solver=MPCC(self.path,self.vehicle)
        self.odom=None;self.phase=None;self.reference=None;self.odom_time=None
        self.control=np.zeros(2);self.active=False;self.solved=None;self.last_actuate=None
        self.pub=self.create_publisher(TwistStamped,'ackermann_steering_controller/reference',10)
        self.status=self.create_publisher(ControlStatus,'control_status',10)
        self.create_subscription(Odometry,'ground_truth',self.on_odom,10)
        self.create_subscription(PhaseState,'phase',lambda m:setattr(self,'phase',m),10)
        self.create_subscription(PhaseReference,'phase_reference',lambda m:setattr(self,'reference',m),10)
        self.solve_group=MutuallyExclusiveCallbackGroup()
        self.actuate_group=ReentrantCallbackGroup()
        self.create_timer(0.1,self.solve,callback_group=self.solve_group)
        self.create_timer(0.01,self.actuate,callback_group=self.actuate_group)

    def now(self):return self.get_clock().now().nanoseconds*1e-9

    def on_odom(self,msg):self.odom=msg;self.odom_time=self.now()

    def fresh(self,msg):
        if msg is None:return False
        age=self.now()-(msg.header.stamp.sec+msg.header.stamp.nanosec*1e-9)
        return msg.valid and 0<=age<=0.2

    def ready(self):
        return (self.odom is not None and self.odom_time is not None
                and 0<=self.now()-self.odom_time<=0.2
                and self.fresh(self.phase) and self.fresh(self.reference))

    def solve(self):
        status=ControlStatus();status.header.stamp=self.get_clock().now().to_msg()
        if not self.ready():
            self.active=False;status.reason='Waiting for fresh state/reference'
        elif self.reference.phase_rate<=0:
            status.success=True;status.reason='Hold: nonpositive phase rate'
            self.active=False
            self.control[:]=0;self.solved=self.now();self.solver.warm=None
        else:
            p=self.odom.pose.pose.position;q=self.odom.pose.pose.orientation
            yaw=math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))
            v=max(0.,self.odom.twist.twist.linear.x)
            state=np.array([p.x,p.y,yaw,v,self.path.s_from_phase(self.phase.phase)])
            self.control,ok,elapsed,reason=self.solver.solve(state,self.reference.phase_rate)
            self.active=ok;self.solved=self.now()
            status.success=bool(ok);status.solve_time_ms=elapsed*1000;status.reason=reason
            _,status.lateral_error=self.path.project([p.x,p.y],self.phase.phase,0,0)
            status.steering=float(self.control[0]);status.throttle=float(self.control[1])
            status.measured_speed=float(v)
            status.requested_speed=float(np.clip(v+0.01*(-self.vehicle.damping*v+
                self.vehicle.alpha*self.control[1]-self.vehicle.beta),0,self.vehicle.max_speed)) if ok else 0.
        self.status.publish(status)

    def actuate(self):
        now=self.now(); dt=0.01 if self.last_actuate is None else now-self.last_actuate
        self.last_actuate=now
        msg=TwistStamped();msg.header.stamp=self.get_clock().now().to_msg()
        if (0<dt<=0.2 and self.active and self.ready() and self.solved is not None
                and 0<=now-self.solved<=0.2):
            p=self.vehicle;v=max(0.,self.odom.twist.twist.linear.x)
            speed=float(np.clip(v+dt*(-p.damping*v+p.alpha*self.control[1]-p.beta),0,p.max_speed))
            msg.twist.linear.x=speed;msg.twist.angular.z=speed*math.tan(self.control[0])/p.wheelbase
        self.pub.publish(msg)


def main(args=None):
    rclpy.init(args=args);node=Controller()
    executor=MultiThreadedExecutor(num_threads=3);executor.add_node(node)
    try:executor.spin()
    except KeyboardInterrupt:pass
    finally:executor.shutdown();node.destroy_node();rclpy.try_shutdown()
