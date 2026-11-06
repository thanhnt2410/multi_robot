"""Behavior checks for paper equations, branch continuity and constrained MPCC."""
import unittest
import numpy as np
from lissajous_path.core import Scenario,Lissajous,ring_rhs,rk4
from patrol_evaluation.metrics import Sensing


class Mathematics(unittest.TestCase):
    def setUp(self):self.c=Scenario(A=1.8,B=.8,omega=.3);self.path=Lissajous(self.c)

    def test_rotating_equilibrium_and_stopped_agent(self):
        theta=np.array([self.c.phase(i) for i in range(1,6)])
        np.testing.assert_allclose(ring_rhs(theta,self.c),.3,atol=1e-14)
        self.assertEqual(ring_rhs(theta,self.c,stopped=(1,5))[0],0)
        theta[0]+=.12;before=np.linalg.norm(ring_rhs(theta,self.c)-.3)
        for _ in range(1000):theta=rk4(lambda x:ring_rhs(x,self.c),theta,.01)
        self.assertLess(np.linalg.norm(ring_rhs(theta,self.c)-.3),before*.01)

    def test_arc_roundtrip_multiple_cycles(self):
        phase=np.linspace(-8*np.pi,8*np.pi,1000)
        np.testing.assert_allclose(self.path.phase_from_s(self.path.s_from_phase(phase)),phase,atol=1e-10)

    def test_projection_crossing_and_wrap(self):
        for center in (np.pi/2,2*np.pi,5*np.pi/2):
            previous=center-.04
            for phase in np.linspace(center-.03,center+.03,30):
                previous,error=self.path.project(self.path.xy(phase),previous,0.,.02)
                self.assertLess(abs(previous-phase),1e-6);self.assertLess(error,1e-6)

    def test_sensing_reflection_and_seed(self):
        a=Sensing(self.c,count=100,seed=42);b=Sensing(self.c,count=100,seed=42)
        np.testing.assert_array_equal(a.targets,b.targets)
        a.update(np.array([[0.,0.]]*5),100.,100.)
        self.assertTrue((np.abs(a.targets)<=np.array([self.c.A,self.c.B])+1e-12).all())

    def test_invalid_topology(self):
        with self.assertRaises(ValueError):Scenario(order=(1,2,3,3,5))


class Optimization(unittest.TestCase):
    def test_stationary_car_can_restart_near_a_tight_turn(self):
        from ackermann_mpcc.solver import MPCC,Vehicle
        path=Lissajous(Scenario(A=43.2,B=19.2,omega=.0125))
        vehicle=Vehicle();phase=10.4342
        tangent=path.derivative(phase)/path.metric(phase)
        xy=path.xy(phase)+.2*np.array([-tangent[1],tangent[0]])
        state=np.r_[xy,path.heading(phase),0.,path.s_from_phase(phase)]
        solver=MPCC(path,vehicle,deadline=.5)
        control,ok,_,reason=solver.solve(state,.06)
        self.assertTrue(ok,reason)
        self.assertGreater(vehicle.alpha*control[1]-vehicle.beta,0.)

    def test_mpcc_bounds_and_finite_prediction(self):
        from ackermann_mpcc.solver import MPCC,Vehicle
        path=Lissajous();v=Vehicle();phase=.6;xy=path.xy(phase)
        solver=MPCC(path,v,deadline=1.)
        z=np.r_[xy,path.heading(phase),.5,path.s_from_phase(phase)]
        u,ok,elapsed,reason=solver.solve(z,.075)
        self.assertTrue(ok,reason);self.assertTrue(np.isfinite(u).all())
        self.assertLessEqual(abs(u[0]),v.max_steering+1e-6)
        self.assertGreaterEqual(u[1],v.throttle_min-1e-6)
        self.assertLessEqual(u[1],v.throttle_max+1e-6)


if __name__=='__main__':unittest.main()
