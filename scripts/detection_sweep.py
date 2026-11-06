#!/usr/bin/env python3
"""Evaluate ideal 15-agent detection with seeds 0..9, without Gazebo."""
import argparse
import json
from pathlib import Path
import subprocess
import sys


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output',type=Path,required=True)
    args=parser.parse_args();args.output.mkdir(parents=True,exist_ok=False)
    summaries=[]
    for seed in range(10):
        output=args.output/f'seed_{seed}'
        with (args.output/f'seed_{seed}.log').open('w') as log:
            subprocess.run([sys.executable,'-m','patrol_evaluation.offline','--case','detection',
                '--duration','10.5','--failure-start','11','--failure-end','12',
                '--seed',str(seed),'--output',str(output)],stdout=log,stderr=subprocess.STDOUT,check=True)
        summary=json.loads((output/'summary.json').read_text());summaries.append(summary)
        print(f'seed {seed}: detected {summary["detected"]}, coverage {summary["coverage"]:.4f}',flush=True)
    (args.output/'summary.json').write_text(json.dumps(summaries,indent=2))


if __name__=='__main__':main()
