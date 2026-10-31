#!/usr/bin/env python3
"""Fit signed affine force calibration from two measured, known axial load states."""
import argparse
import json
import math
import statistics
from pathlib import Path

GF_TO_N=0.00980665


def raw_means(run):
    path=Path(run)
    if path.is_dir(): path=path/'telemetry.jsonl'
    columns=[[] for _ in range(6)]
    with path.open() as stream:
        for line in stream:
            row=json.loads(line)
            if row.get('kind')!='feedback': continue
            data=row['data']
            if len(data['force_raw'])!=6 or not all(data['valid']) or any(data['fault_codes']):
                continue
            for values,value in zip(columns,data['force_raw']): values.append(float(value))
    if any(not values for values in columns): raise ValueError('no valid six-axis force samples')
    return [statistics.mean(v) for v in columns],[statistics.pstdev(v) for v in columns],len(columns[0])


def affine_calibration(first_raw,second_raw,first_force_n,second_force_n):
    arrays=(first_raw,second_raw,first_force_n,second_force_n)
    if any(len(v)!=6 or not all(math.isfinite(float(x)) for x in v) for v in arrays):
        raise ValueError('six finite values required for each calibration state')
    zero=[]; scale=[]
    for i,(r0,r1,f0,f1) in enumerate(zip(*arrays)):
        if r1==r0 or f1==f0:
            raise ValueError(f'axis {i+1}: two distinct known force and raw values required')
        slope=(f1-f0)/(r1-r0)
        intercept_zero=r0-f0/slope
        if not math.isfinite(slope) or not math.isfinite(intercept_zero):
            raise ValueError('nonfinite calibration')
        scale.append(slope); zero.append(intercept_zero)
    return {'raw_zero':zero,'newtons_per_raw':scale}


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--config',required=True)
    p.add_argument('--first-run',required=True)
    p.add_argument('--second-run',required=True)
    p.add_argument('--first-force-n',type=float,nargs=6,required=True,metavar='N')
    p.add_argument('--second-force-n',type=float,nargs=6,required=True,metavar='N')
    p.add_argument('--output',required=True,help='new runtime config file; existing files are not overwritten')
    a=p.parse_args()
    first,noise0,n0=raw_means(a.first_run); second,noise1,n1=raw_means(a.second_run)
    c=json.loads(Path(a.config).read_text())
    c['calibration']=affine_calibration(first,second,a.first_force_n,a.second_force_n)
    c['calibration']['measurement']={'first_run':str(a.first_run),'second_run':str(a.second_run),
        'first_known_force_n':a.first_force_n,'second_known_force_n':a.second_force_n,
        'raw_mean_first':first,'raw_mean_second':second,'raw_std_first':noise0,'raw_std_second':noise1,
        'sample_counts':[n0,n1],
        'positive_force':'user supplied signed axial forces in the chosen extension-force convention'}
    with Path(a.output).open('x') as stream: json.dump(c,stream,indent=2,allow_nan=False); stream.write('\n')
    print(json.dumps(c['calibration'],indent=2))


if __name__=='__main__': main()
