#!/usr/bin/env python3
"""Positive sensor check: create a static robot_99 fixture intersecting one car.

Requires a dedicated fleet:=single control:=false simulation. Do not run in a
mission world: the fixture is intentionally colliding and remains until shutdown.
"""
import argparse
import json
import re
import subprocess
import time

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from ros_gz_interfaces.msg import Contacts


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output',default='/tmp/patrol_contacts_validation.json')
    args=parser.parse_args();rclpy.init();node=Node('validate_contacts')
    latest=[];pairs=[];raw=[0]
    node.create_subscription(Odometry,'/robot_2/ground_truth',lambda m:latest.append(m),10)
    def contact(msg):
        raw[0]+=len(msg.contacts)
        for c in msg.contacts:
            names=c.collision1.name+' '+c.collision2.name
            if {'robot_2','robot_99'}.issubset(set(re.findall(r'robot_\d+',names))):pairs.append(names)
    node.create_subscription(Contacts,'/robot_2/contacts',contact,10)
    deadline=time.monotonic()+30
    while not latest and time.monotonic()<deadline:rclpy.spin_once(node,timeout_sec=.1)
    if not latest:raise RuntimeError('Dedicated single-car simulation not ready')
    sdf='<sdf version="1.6"><model name="robot_99"><static>true</static><link name="body"><collision name="robot_99_collision"><geometry><box><size>0.3 0.3 0.3</size></box></geometry></collision></link></model></sdf>'
    subprocess.run(['ros2','run','ros_gz_sim','create','-world','patrol','-name','robot_99',
                    '-string',sdf,'-x','.2','-y','0','-z','.12'],check=True,timeout=30)
    deadline=time.monotonic()+10
    while not pairs and time.monotonic()<deadline:rclpy.spin_once(node,timeout_sec=.1)
    result={'passed':bool(pairs),'raw_contact_records':raw[0],'robot_contact_pairs':pairs[:5]}
    with open(args.output,'w') as f:json.dump(result,f,indent=2)
    print(json.dumps(result,indent=2));node.destroy_node();rclpy.shutdown()
    if not result['passed']:raise SystemExit(1)


if __name__=='__main__':main()
