"""Deterministic sampled sensing and persistent-monitoring metrics."""
import numpy as np


class Sensing:
    def __init__(self,c,count=10000,seed=42,speed_min=0.2,speed_max=1.0):
        self.c=c;self.radius=1.01*max(c.bounds()['coverage_radius'],c.bounds()['detection_radius'])
        self.rng=np.random.default_rng(seed)
        self.targets=self.rng.uniform([-c.A,-c.B],[c.A,c.B],(count,2))
        angle=self.rng.uniform(0,2*np.pi,count);speed=self.rng.uniform(speed_min,speed_max,count)
        self.velocity=speed[:,None]*np.stack((np.cos(angle),np.sin(angle)),axis=-1)
        self.detected=np.zeros(count,dtype=bool);self.detection_times=np.full(count,np.nan)
        x,y=np.meshgrid(np.linspace(-c.A,c.A,101),np.linspace(-c.B,c.B,81))
        self.grid=np.stack((x.ravel(),y.ravel()),axis=-1)
        n=len(c.order);self.covered=np.zeros((n,len(self.grid)),dtype=bool)
        self.last_visit=np.full((n,len(self.grid)),np.nan);self.revisit_max=0.

    def update(self,xy,t,dt):
        bounds=np.array([self.c.A,self.c.B]); width=2*bounds
        # Triangle-wave reflection also handles more than one boundary crossing.
        raw=self.targets+self.velocity*dt
        cell=np.floor((raw+bounds)/width).astype(int)
        folded=np.mod(raw+bounds,2*width)
        self.targets=np.where(folded<=width,folded,2*width-folded)-bounds
        self.velocity*=np.where(cell % 2 == 0,1,-1)
        distances=np.linalg.norm(xy[:,None,:]-self.targets[None,:,:],axis=-1)
        detected=(distances<=self.radius).any(axis=0)
        new=detected & ~self.detected;self.detection_times[new]=t;self.detected|=detected
        seen=np.linalg.norm(xy[:,None,:]-self.grid[None,:,:],axis=-1)<=self.radius
        # Revisit is measured on entry after a sampled absence, not every in-disk sample.
        if not hasattr(self,'previous_seen'):self.previous_seen=np.zeros_like(seen)
        entered=seen & ~self.previous_seen
        revisited=entered & np.isfinite(self.last_visit)
        if revisited.any():self.revisit_max=max(self.revisit_max,float(np.max(t-self.last_visit[revisited])))
        self.last_visit[seen]=t;self.covered|=seen;self.previous_seen=seen
        return dict(detected=int(self.detected.sum()),coverage=float(self.covered.any(axis=0).mean()),
                    coverage_per_agent=self.covered.mean(axis=1).tolist(),revisit_max=self.revisit_max)
