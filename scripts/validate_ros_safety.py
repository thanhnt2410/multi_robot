#!/usr/bin/env python3
"""ROS-node fault checks; run in an isolated ROS_DOMAIN_ID with local DDS access."""
from types import SimpleNamespace
import unittest
from unittest.mock import patch
import numpy as np
import rclpy
from nav_msgs.msg import Odometry
from swarm_interfaces.msg import PhaseState,PhaseReference
from ackermann_mpcc.node import Controller
from swarm_coordination.agent import Agent


class FaultChecks(unittest.TestCase):
    def setUp(self):rclpy.init()
    def tearDown(self):rclpy.try_shutdown()

    def test_stale_state_failed_solver_hold_and_clock_rewind(self):
        with patch('ackermann_mpcc.node.MPCC') as mock:
            mock.return_value.solve.return_value=(np.array([.1,.1]),True,.01,'test_success')
            node=Controller()
        try:
            now=[100.];node.now=lambda:now[0]
            commands=[];statuses=[]
            node.pub=SimpleNamespace(publish=commands.append)
            node.status=SimpleNamespace(publish=statuses.append)
            def ready():
                node.odom=Odometry();node.odom.pose.pose.orientation.w=1.
                node.odom.twist.twist.linear.x=.5;node.odom_time=now[0]
                xy=node.path.xy(.6);node.odom.pose.pose.position.x=float(xy[0]);node.odom.pose.pose.position.y=float(xy[1])
                node.phase=PhaseState(valid=True,phase=.6);node.reference=PhaseReference(valid=True,phase_rate=.075)
                for msg in (node.phase,node.reference):msg.header.stamp.sec=int(now[0]);msg.header.stamp.nanosec=int((now[0]%1)*1e9)
            ready();node.solve();now[0]+=.01;node.actuate()
            self.assertGreater(commands[-1].twist.linear.x,0)
            now[0]+=.3;node.actuate();self.assertEqual(commands[-1].twist.linear.x,0)
            ready();node.solver.solve.return_value=(np.zeros(2),False,.1,'Maximum_CpuTime_Exceeded')
            node.solve();now[0]+=.01;node.actuate()
            self.assertFalse(statuses[-1].success);self.assertEqual(commands[-1].twist.linear.x,0)
            ready();node.reference.phase_rate=0.0;node.solve();now[0]+=.01;node.actuate()
            self.assertEqual(commands[-1].twist.linear.x,0)
            now[0]=1.;node.actuate();self.assertEqual(commands[-1].twist.linear.x,0)
        finally:node.destroy_node()

    def test_agent_clock_reset_discards_old_neighbors(self):
        node=Agent()
        try:
            node.virtual=True;node.valid=True;node.theta=42.;node.last=100.;node.epoch=50.
            node.neighbors={1:PhaseState(valid=True)};node.now=lambda:1.
            phases=[];references=[]
            node.pub=SimpleNamespace(publish=phases.append);node.ref=SimpleNamespace(publish=references.append)
            node.tick()
            self.assertAlmostEqual(node.theta,node.c.phase(node.id))
            self.assertEqual(node.neighbors,{})
            self.assertIsNone(node.epoch);self.assertFalse(references[-1].valid)
        finally:node.destroy_node()

    def test_agent_retains_progress_after_odometry_gap(self):
        node=Agent()
        try:
            actual=node.c.phase(node.id)+.35
            node.theta=actual;node.last_odom=100.;node.now=lambda:101.
            message=Odometry();xy=node.path.xy(actual)
            message.pose.pose.position.x=float(xy[0]);message.pose.pose.position.y=float(xy[1])
            message.pose.pose.orientation.w=1.
            node.odometry(message)
            self.assertAlmostEqual(node.theta,actual,places=5)
        finally:node.destroy_node()


if __name__=='__main__':unittest.main()
