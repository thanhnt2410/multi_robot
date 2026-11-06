#!/usr/bin/env python3
"""Compare a MATLAB export to Python's ideal-feedback patrol CSV.

Reference columns: t, theta_1, ..., theta_5 (no header).
Python CSV comes from: offline --paper --duration 45.
"""
import argparse
import csv
import json
from pathlib import Path

import numpy as np


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('reference',type=Path)
    parser.add_argument('python_samples',type=Path)
    parser.add_argument('--output',type=Path,default=Path('/tmp/patrol_matlab_comparison.json'))
    args=parser.parse_args();reference=np.loadtxt(args.reference,delimiter=',')
    if reference.ndim!=2 or reference.shape[1]!=6:parser.error('Expected MATLAB columns t, theta_1,...,theta_5')
    with args.python_samples.open() as f:rows=list(csv.DictReader(f))
    errors=[]
    for agent in range(1,6):
        data=np.array([[float(r['time']),float(r['phase'])] for r in rows if int(r['agent'])==agent])
        if not len(data) or reference[0,0]<data[0,0]-1e-9 or reference[-1,0]>data[-1,0]+1e-9:
            parser.error('Python samples do not cover the full MATLAB interval')
        prediction=np.interp(reference[:,0],data[:,0],data[:,1])
        errors.append(float(np.max(np.abs(prediction-reference[:,agent]))))
    result={'max_phase_error_per_agent':errors,'threshold_rad':1e-3,'passed':max(errors)<1e-3}
    args.output.write_text(json.dumps(result,indent=2)+'\n');print(json.dumps(result,indent=2))
    if not result['passed']:raise SystemExit(1)


if __name__=='__main__':main()
