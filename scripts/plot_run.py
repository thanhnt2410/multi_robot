#!/usr/bin/env python3
"""Export Gazebo experiment figures without needing a live ROS graph."""
import argparse
import csv
import json
import os
from pathlib import Path

os.environ.setdefault('MPLCONFIGDIR','/tmp/patrol_matplotlib')
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np
from lissajous_path.core import Scenario,Lissajous


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('run',type=Path);parser.add_argument('--output',type=Path)
    args=parser.parse_args();metadata=json.loads((args.run/'metadata.json').read_text())
    path=Lissajous(Scenario(**metadata['scenario']))
    with (args.run/'samples.csv').open() as f:rows=list(csv.DictReader(f))
    data={i:[r for r in rows if int(r['agent'])==i] for i in path.c.order}
    fig,axes=plt.subplots(2,2,figsize=(12,8))
    axes[0,0].plot(*path.xy(path.gamma).T,':',color='gray',linewidth=1)
    for i,agent in data.items():
        number=lambda key:np.array([float(r[key]) for r in agent])
        axes[0,0].plot(number('x'),number('y'),label=f'{i}'+(' (virtual)' if i not in metadata['physical_ids'] else ''))
        axes[0,1].plot(number('time'),number('phase'),label=str(i))
        if i in metadata['physical_ids']:axes[1,0].plot(number('time'),number('lateral_error'),label=str(i))
    first=data[path.c.order[0]];t=np.array([float(r['time']) for r in first])
    axes[1,1].plot(t,[float(r['min_distance']) for r in first])
    axes[0,0].set(xlabel='x [m]',ylabel='y [m]',title='Actual trajectories');axes[0,0].axis('equal');axes[0,0].legend()
    axes[0,1].set(xlabel='Time [s]',ylabel='Phase [rad]',title='Unwrapped measured/virtual phases')
    axes[1,0].set(xlabel='Time [s]',ylabel='Local path error [m]');axes[1,0].legend()
    axes[1,1].set(xlabel='Time [s]',ylabel='Minimum pair distance [m]')
    for ax in (axes[0,1],axes[1,0],axes[1,1]):
        start=metadata.get('failure_start',float('inf'))
        end=metadata.get('failure_end',float('inf'))
        if 0<=start<=t.max():ax.axvspan(start,min(end,t.max()),color='orange',alpha=.12)
        perturb=metadata.get('perturb_time',-1.)
        if 0<=perturb<=t.max():ax.axvline(perturb,color='red',linestyle='--',alpha=.6)
    for ax in axes.ravel():ax.grid(True,alpha=.3)
    fig.tight_layout();output=args.output or args.run/'overview.png';fig.savefig(output,dpi=150);plt.close(fig)
    print(output)


if __name__=='__main__':main()
