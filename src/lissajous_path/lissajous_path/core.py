"""ROS-independent paper equations; phase is always unwrapped."""
from dataclasses import dataclass
import math

import numpy as np
from scipy.integrate import cumulative_trapezoid
from scipy.optimize import minimize_scalar


@dataclass(frozen=True)
class Scenario:
    A: float = 7.2
    B: float = 3.2
    a: int = 3
    b: int = 2
    omega: float = 0.075
    order: tuple = (1, 2, 5, 3, 4)
    p: int = 2

    def __post_init__(self):
        n = len(self.order)
        if (self.A <= 0 or self.B <= 0 or self.omega <= 0 or self.b <= 0
                or self.a <= 0 or self.a % 2 != 1 or math.gcd(self.a, self.b) != 1
                or self.a + self.b != n or sorted(self.order) != list(range(1, n+1))
                or not n/4 < self.p < 3*n/4):
            raise ValueError('Invalid Lissajous/ring/stable-equilibrium configuration')

    def phase(self, agent):
        return 2*np.pi*self.p*self.order.index(agent)/len(self.order)

    def neighbors(self, agent):
        k = self.order.index(agent)
        return self.order[k-1], self.order[(k+1) % len(self.order)]

    def bounds(self):
        n = len(self.order)
        clusters = math.gcd(n, self.p)
        return dict(robot_radius=np.sin(np.pi/n)*self.A*self.B /
                    np.hypot(self.A*self.a, self.B*self.b),
                    coverage_radius=max(self.B*np.sin(np.pi/(2*self.a)),
                                        self.A*np.sin(np.pi/(2*self.b))),
                    detection_radius=np.sin(np.pi/(n/clusters))*np.hypot(self.A,self.B),
                    detection_time=2*np.pi/(self.omega*(n/clusters)))


def phase_rate(phase, neighbors, omega, feedback=True):
    return float(omega - np.sin(np.asarray(neighbors)-phase).sum()) if feedback else omega


def ring_rhs(theta, scenario, feedback=True, stopped=()):
    return np.array([0.0 if i in stopped else phase_rate(theta[i-1],
        [theta[j-1] for j in scenario.neighbors(i)], scenario.omega, feedback)
        for i in range(1, len(theta)+1)])


class Lissajous:
    def __init__(self, scenario=Scenario(), samples=30001):
        self.c = scenario
        self.gamma = np.linspace(0, 2*np.pi, samples)
        self.arc = cumulative_trapezoid(self.metric(self.gamma), self.gamma, initial=0)
        self.length = self.arc[-1]

    def xy(self, phase):
        c = self.c
        return np.stack((c.A*np.cos(c.a*phase), c.B*np.sin(c.b*phase)), axis=-1)

    def derivative(self, phase):
        c = self.c
        return np.stack((-c.A*c.a*np.sin(c.a*phase), c.B*c.b*np.cos(c.b*phase)), axis=-1)

    def metric(self, phase):
        return np.linalg.norm(self.derivative(phase), axis=-1)

    def heading(self, phase):
        d = self.derivative(phase)
        return np.arctan2(d[..., 1], d[..., 0])

    def s_from_phase(self, phase):
        turns = np.floor(np.asarray(phase)/(2*np.pi))
        return turns*self.length+np.interp(np.mod(phase,2*np.pi),self.gamma,self.arc)

    def phase_from_s(self, s):
        turns = np.floor(np.asarray(s)/self.length)
        return turns*2*np.pi+np.interp(np.mod(s,self.length),self.arc,self.gamma)

    def project(self, xy, previous, speed, dt, window=0.15):
        predicted = previous + speed*dt/float(self.metric(previous))
        # Local continuity disambiguates the self-intersections of the curve.
        result = minimize_scalar(lambda g: float(np.sum((self.xy(g)-xy)**2)),
                                 bounds=(predicted-window,predicted+window),
                                 method='bounded', options={'xatol':1e-10})
        return float(result.x), float(np.sqrt(result.fun))

    def curvature(self, phase):
        c=self.c;d=self.derivative(phase)
        dd=np.stack((-c.A*c.a**2*np.cos(c.a*phase),-c.B*c.b**2*np.sin(c.b*phase)),axis=-1)
        return (d[...,0]*dd[...,1]-d[...,1]*dd[...,0])/self.metric(phase)**3

    def max_curvature(self):
        return float(np.max(np.abs(self.curvature(self.gamma))))


def rk4(f, state, dt):
    k1=f(state); k2=f(state+dt*k1/2); k3=f(state+dt*k2/2); k4=f(state+dt*k3)
    return state+dt*(k1+2*k2+2*k3+k4)/6


def wrap(angle):
    return (np.asarray(angle)+np.pi) % (2*np.pi)-np.pi
