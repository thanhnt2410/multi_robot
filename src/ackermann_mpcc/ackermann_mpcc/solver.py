"""MPCC with the paper's throttle model and numerical arc-length inverse."""
from dataclasses import dataclass
import time

import numpy as np


@dataclass
class Vehicle:
    wheelbase: float = 0.285
    damping: float = 0.94
    alpha: float = 36.8
    beta: float = 0.94
    max_steering: float = np.pi/9
    throttle_min: float = 0.0
    throttle_max: float = 0.25
    max_speed: float = 2.5


class MPCC:
    def __init__(self, path, vehicle=None, horizon=15, dt=0.1, deadline=0.1):
        import casadi as ca
        self.ca=ca; self.path=path; self.v=vehicle or Vehicle()
        self.M=horizon; self.dt=dt; self.deadline=deadline; self.warm=None
        v=self.v; c=path.c
        # Smooth inverse on an extended domain avoids the derivative jumps of a
        # piecewise-linear lookup and a symbolic modulo inside the optimizer.
        extension=max(2,int(np.ceil(v.max_speed*horizon*dt/path.length))+1)
        s_grid=np.linspace(-path.length,extension*path.length,3001*(extension+1))
        inverse=ca.interpolant('inverse_arc','bspline',[s_grid],path.phase_from_s(s_grid))
        z=ca.MX.sym('z',5); u=ca.MX.sym('u',2)
        rhs=ca.vertcat(z[3]*ca.cos(z[2]),z[3]*ca.sin(z[2]),
                      z[3]*ca.tan(u[0])/v.wheelbase,
                      -v.damping*z[3]+v.alpha*u[1]-v.beta,z[3])
        f=ca.Function('bicycle',[z,u],[rhs])
        U=ca.MX.sym('U',2,horizon); P=ca.MX.sym('P',6)
        current=P[:5]; cost=0; speeds=[]
        for k in range(horizon):
            gamma=inverse(current[4])
            tangent=ca.vertcat(-c.A*c.a*ca.sin(c.a*gamma),c.B*c.b*ca.cos(c.b*gamma))
            metric=ca.norm_2(tangent); tangent=tangent/metric
            error=current[:2]-ca.vertcat(c.A*ca.cos(c.a*gamma),c.B*ca.sin(c.b*gamma))
            lag=ca.dot(error,tangent)
            lat=ca.dot(error,ca.vertcat(-tangent[1],tangent[0]))
            cost+=(P[5]-current[3]/metric)**2+0.01*lag**2+0.025*lat**2+0.01*ca.sumsqr(U[:,k])
            k1=f(current,U[:,k]);k2=f(current+dt*k1/2,U[:,k])
            k3=f(current+dt*k2/2,U[:,k]);k4=f(current+dt*k3,U[:,k])
            current=current+dt*(k1+2*k2+2*k3+k4)/6
            speeds.append(current[3])
        self.solver=ca.nlpsol('mpcc','ipopt',dict(x=ca.vec(U),p=P,f=cost,g=ca.vertcat(*speeds)),
            {'print_time':False,'ipopt.print_level':0,'ipopt.sb':'yes',
             'ipopt.max_iter':60,'ipopt.tol':1e-4,
             # CPU time includes the other ROS callback threads in this process.
             # Enforce the control deadline using elapsed wall time instead.
             'ipopt.max_wall_time':deadline*.9,'ipopt.max_cpu_time':deadline*4})
        self.lower=np.tile([-v.max_steering,v.throttle_min],horizon)
        self.upper=np.tile([v.max_steering,v.throttle_max],horizon)

    def initial_controls(self, state, desired_rate):
        """Start with forward progress and path curvature, including at rest.

        Zero steering and zero predicted speed leave the contouring problem
        poorly conditioned after a stop. A feasible acceleration ramp gives
        IPOPT a moving trajectory from which to optimize steering.
        """
        v=self.v;speed=max(0.,float(state[3]));progress=float(state[4]);controls=[]
        for _ in range(self.M):
            phase=self.path.phase_from_s(progress)
            target=np.clip(desired_rate*self.path.metric(phase),0.,v.max_speed)
            acceleration=(target-speed)/self.dt
            throttle=float(np.clip((acceleration+v.damping*speed+v.beta)/v.alpha,
                                   v.throttle_min,v.throttle_max))
            steering=float(np.clip(np.arctan(v.wheelbase*self.path.curvature(phase)),
                                   -v.max_steering,v.max_steering))
            controls.extend((steering,throttle))
            force=v.alpha*throttle-v.beta
            following=(speed*np.exp(-v.damping*self.dt)+force/v.damping*(1-np.exp(-v.damping*self.dt))
                       if v.damping else speed+self.dt*force)
            following=max(0.,following)
            progress+=self.dt*(speed+following)/2;speed=following
        return np.array(controls)

    def solve(self, state, desired_rate):
        v=self.v
        state=np.array(state,dtype=float,copy=True)
        state[4] %= self.path.length
        if self.warm is None:
            initial=self.initial_controls(state,desired_rate)
        else: initial=self.warm
        start=time.perf_counter()
        try:
            result=self.solver(x0=initial,p=np.r_[state,desired_rate],lbx=self.lower,ubx=self.upper,
                               lbg=np.zeros(self.M),ubg=np.full(self.M,v.max_speed))
            elapsed=time.perf_counter()-start
            stats=self.solver.stats(); controls=np.array(result['x']).ravel().reshape(self.M,2)
            success=bool(stats['success']) and np.isfinite(controls).all() and elapsed<=self.deadline
            if success:self.warm=np.vstack((controls[1:],controls[-1:])).ravel()
            else:self.warm=None
            return controls[0] if success else np.zeros(2),success,elapsed,str(stats['return_status'])
        except RuntimeError as exc:
            self.warm=None
            return np.zeros(2),False,time.perf_counter()-start,str(exc)
