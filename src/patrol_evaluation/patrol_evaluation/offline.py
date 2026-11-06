"""Numerical reference and MPCC experiments without Gazebo or ROS nodes."""
import argparse
import csv
from dataclasses import asdict
import json
import os
from pathlib import Path

os.environ.setdefault('OPENBLAS_NUM_THREADS','1')
os.environ.setdefault('OMP_NUM_THREADS','1')
import numpy as np
from lissajous_path.core import Scenario,Lissajous,ring_rhs,rk4,wrap
from .metrics import Sensing


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--mode',choices=['ideal','mpcc'],default='ideal')
    parser.add_argument('--case',choices=['patrol','detection'],default='patrol')
    parser.add_argument('--paper',action='store_true')
    parser.add_argument('--feedback-off',action='store_true')
    parser.add_argument('--duration',type=float,default=45.)
    parser.add_argument('--failure-start',type=float,default=9.6)
    parser.add_argument('--failure-end',type=float,default=19.5)
    parser.add_argument('--seed',type=int,default=42)
    parser.add_argument('--output',type=Path,required=True)
    args=parser.parse_args()
    if args.duration<=0:parser.error('duration must be positive')
    if args.case=='detection' and args.mode=='mpcc':parser.error('detection case is the 15-agent ideal model')
    args.output.mkdir(parents=True,exist_ok=False)
    c=Scenario(A=1.8,B=.8,omega=.3) if args.paper else Scenario()
    if args.case=='detection':c=Scenario(A=5.,B=5.,a=7,b=8,omega=.05,order=tuple(range(1,16)),p=7)
    n=len(c.order)
    path=Lissajous(c);theta=np.array([c.phase(i) for i in range(1,n+1)])
    initial=theta.copy();sensing=Sensing(c,seed=args.seed)
    dt=.01 if args.mode=='ideal' else .1
    t=np.unique(np.r_[np.arange(0,args.duration,dt),args.duration,
          [e for e in [args.failure_start,args.failure_end] if 0<=e<=args.duration]])
    states=np.zeros((n,5));states[:,:2]=path.xy(theta);states[:,2]=path.heading(theta)
    states[:,3]=c.omega*path.metric(theta);states[:,4]=path.s_from_phase(theta)
    solvers={};solve_times=[];failures=0;failure_reasons={}
    if args.mode=='mpcc':
        from ackermann_mpcc.solver import MPCC,Vehicle
        vehicle=Vehicle(wheelbase=.175,throttle_min=.1,max_speed=10.) if args.paper else Vehicle()
        # Numerical experiments permit 1s solver time; real-time feasibility is measured separately.
        solvers={i:MPCC(path,vehicle,deadline=1.) for i in (2,3,4)}
    rows=[];minimum=float('inf');max_error=0.
    for k,now in enumerate(t):
        xy=path.xy(theta) if args.mode=='ideal' else states[:,:2].copy()
        d=np.linalg.norm(xy[:,None,:]-xy[None,:,:],axis=-1);np.fill_diagonal(d,np.inf)
        minimum=min(minimum,float(d.min()))
        sense=sensing.update(xy,float(now),0 if k==0 else float(now-t[k-1]))
        for i in range(n):
            _,error=path.project(xy[i],theta[i],0,0);max_error=max(max_error,error)
            rows.append([now,i+1,*xy[i],theta[i],error])
        if k==len(t)-1:break
        h=t[k+1]-now;mid=now+h/2
        stopped=(1,5) if args.failure_start<=mid<args.failure_end else ()
        if args.mode=='ideal':theta=rk4(lambda z:ring_rhs(z,c,not args.feedback_off,stopped),theta,h)
        else:
            rates=ring_rhs(theta,c,not args.feedback_off,stopped)
            for i in range(1,n+1):
                j=i-1
                if i not in solvers:
                    theta[j]+=h*rates[j];states[j,:2]=path.xy(theta[j]);continue
                u,ok,elapsed,reason=solvers[i].solve(states[j],rates[j]);solve_times.append(elapsed*1000)
                if not ok:
                    failures+=1;failure_reasons[reason]=failure_reasons.get(reason,0)+1
                    states[j,3]=0;continue
                def f(z):return np.array([z[3]*np.cos(z[2]),z[3]*np.sin(z[2]),
                    z[3]*np.tan(u[0])/vehicle.wheelbase,
                    -vehicle.damping*z[3]+vehicle.alpha*u[1]-vehicle.beta,z[3]])
                states[j]=rk4(f,states[j],h)
                theta[j]=path.phase_from_s(states[j,4])
    with (args.output/'samples.csv').open('w',newline='') as f:
        writer=csv.writer(f);writer.writerow(['time','agent','x','y','phase','lateral_error']);writer.writerows(rows)
    summary={**sense,'min_pair_distance':minimum,'tracking_max':max_error,'failed_solves':failures,
             'failure_reasons':failure_reasons,
             'solve_ms':dict(zip(('p50','p95','max'),np.percentile(solve_times,[50,95,100]).tolist())) if solve_times else {},
             'scenario':asdict(c),'mode':args.mode,'seed':args.seed,'feedback':not args.feedback_off,
             'failure_window':[args.failure_start,args.failure_end],'duration':args.duration}
    (args.output/'summary.json').write_text(json.dumps(summary,indent=2))
    np.savez_compressed(args.output/'sensing.npz',detection_times=sensing.detection_times,covered=sensing.covered)
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    data=np.array(rows);fig,axes=plt.subplots(1,2,figsize=(10,4))
    axes[0].plot(*path.xy(path.gamma).T,':',color='gray')
    for i in range(1,n+1):
        z=data[data[:,1]==i];axes[0].plot(z[:,2],z[:,3],label=str(i));axes[1].plot(z[:,0],z[:,4])
    axes[0].axis('equal');axes[0].legend();axes[0].set_xlabel('x [m]');axes[0].set_ylabel('y [m]')
    axes[1].set_xlabel('Time [s]');axes[1].set_ylabel('Unwrapped phase [rad]')
    fig.tight_layout();fig.savefig(args.output/'trajectories.png',dpi=150);plt.close(fig)
    print(json.dumps(summary,indent=2))


if __name__=='__main__':main()
