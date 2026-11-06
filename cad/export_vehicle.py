"""Editable reference CAD matching the URDF primitives, in millimetres.

This is a geometry development starting point, not a fabrication-ready design.
Run with cadquery==2.5.2 in a separate environment from ROS.
"""
import argparse
import json
from pathlib import Path

import cadquery as cq


def build(dimensions):
    d={k:v*1000 for k,v in dimensions.items() if 'mass' not in k}
    assembly=cq.Assembly(name='ackermann_2wd_reference')
    chassis=cq.Workplane('XY').box(d['body_length'],d['body_width'],d['body_height'])
    chassis=chassis.translate((d['wheelbase']/2,0,d['wheel_radius']+d['body_height']/2))
    assembly.add(chassis,name='chassis',color=cq.Color(.25,.35,.65))
    for axle,x in [('rear',0),('front',d['wheelbase'])]:
        for side,sign in [('left',1),('right',-1)]:
            y=sign*d['track_width']/2
            wheel=cq.Solid.makeCylinder(d['wheel_radius'],d['wheel_thickness'],
                cq.Vector(x,y-d['wheel_thickness']/2,d['wheel_radius']),cq.Vector(0,1,0))
            assembly.add(wheel,name=f'{axle}_{side}_wheel',color=cq.Color(.12,.12,.12))
            if axle=='front':
                hub=cq.Solid.makeCylinder(10,30,cq.Vector(x,y-15,d['wheel_radius']),cq.Vector(0,1,0))
                assembly.add(hub,name=f'front_{side}_steering_hub',color=cq.Color(.8,.15,.1))
    return assembly


def main():
    root=Path(__file__).resolve().parent.parent
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--dimensions',type=Path,default=root/'src/ackermann_description/config/vehicle.json')
    parser.add_argument('--output',type=Path,default=root/'cad/ackermann_reference.step')
    args=parser.parse_args();assembly=build(json.loads(args.dimensions.read_text()))
    assembly.save(str(args.output))
    print(f'Exported {args.output}')


if __name__=='__main__':main()
