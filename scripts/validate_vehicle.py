#!/usr/bin/env python3
"""Run against fleet:=single control:=false, in an isolated ROS domain.

Sends low-speed straight and left/right circle commands; exits nonzero if
ground-truth velocity/yaw or the steering geometry violates acceptance limits.
"""
import argparse
import json
import math
import time

import numpy as np
import rclpy
from geometry_msgs.msg import TwistStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import JointState
from rclpy.node import Node


def main():
    parser=argparse.ArgumentParser();parser.add_argument('--output',default='/tmp/patrol_vehicle_validation.json')
    args=parser.parse_args();rclpy.init();node=Node('validate_vehicle')
    odom=[];joints=[]
    node.create_subscription(Odometry,'/robot_2/ground_truth',lambda m:odom.append(m),10)
    node.create_subscription(JointState,'/robot_2/joint_states',lambda m:joints.append(m),10)
    pub=node.create_publisher(TwistStamped,'/robot_2/ackermann_steering_controller/reference',10)
    deadline=time.monotonic()+30
    while (not odom or not joints or pub.get_subscription_count()==0) and time.monotonic()<deadline:
        rclpy.spin_once(node,timeout_sec=.1)
    if not odom or not joints or not pub.get_subscription_count():raise RuntimeError('Vehicle state/controller not ready')
    result={};passed=True
    try:
        for name,speed,yaw_rate in [('straight',.4,0.),('left',.4,.4),('right',.4,-.4),('stop',0.,0.)]:
            start=time.monotonic();samples=[]
            while time.monotonic()-start<6:
                msg=TwistStamped();msg.header.stamp=odom[-1].header.stamp
                msg.twist.linear.x=speed;msg.twist.angular.z=yaw_rate;pub.publish(msg)
                rclpy.spin_once(node,timeout_sec=.02)
                if time.monotonic()-start>3:samples.append((odom[-1].twist.twist.linear.x,odom[-1].twist.twist.angular.z))
            mean=np.mean(samples,axis=0);data=dict(speed=float(mean[0]),yaw_rate=float(mean[1]))
            if speed:
                data['speed_relative_error']=abs(mean[0]-speed)/speed
                ok=data['speed_relative_error']<.1
                if yaw_rate:
                    data['yaw_relative_error']=abs(mean[1]-yaw_rate)/abs(yaw_rate)
                    data['radius']=abs(float(mean[0]/mean[1]))
                    ok=ok and data['yaw_relative_error']<.1
                else:ok=ok and abs(mean[1])<.03
            else:ok=abs(mean[0])<.03 and abs(mean[1])<.03
            data['passed']=bool(ok);passed=passed and ok
            data['joint_positions']=dict(zip(joints[-1].name,joints[-1].position))
            data['joint_velocities']=dict(zip(joints[-1].name,joints[-1].velocity))
            result[name]=data
        result['passed']=bool(passed)
        with open(args.output,'w') as f:json.dump(result,f,indent=2)
        print(json.dumps(result,indent=2))
    finally:
        msg=TwistStamped();msg.header.stamp=odom[-1].header.stamp;pub.publish(msg)
        node.destroy_node();rclpy.shutdown()
    if not passed:raise SystemExit(1)


if __name__=='__main__':main()
