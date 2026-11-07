"""Parametric mechanical layout in mm, authored for this project.

Geometric assembly for simulation and design development. Purchased components
are envelope models, not vendor drawings. Dynamics remain in vehicle.urdf.xacro.
"""
from dataclasses import dataclass
from math import cos, sin, pi

import cadquery as cq


COLORS = {
    'frame': (.12, .16, .20), 'aluminium': (.72, .76, .80),
    'blue': (.06, .36, .72), 'rubber': (.055, .065, .075),
    'gold': (.85, .60, .18), 'red': (.8, .10, .07),
    'green': (.05, .38, .20), 'battery': (.12, .22, .40),
    'white': (.90, .93, .96),
}


@dataclass
class Part:
    name: str
    group: str
    shape: cq.Shape
    material: str
    note: str = ''


def box(x, y, z, center):
    return cq.Workplane('XY').box(x, y, z).val().translate(center)


def cylinder(radius, length, start, direction=(0, 0, 1)):
    return cq.Solid.makeCylinder(radius, length, cq.Vector(*start), cq.Vector(*direction))


def ring(outer, inner, length, start, direction=(0, 0, 1)):
    return cylinder(outer, length, start, direction).cut(cylinder(inner, length, start, direction))


def parts(dimensions):
    d = {k: v*1000 for k, v in dimensions.items() if 'mass' not in k}
    L, W, H = d['body_length'], d['body_width'], d['body_height']
    wb, track, R, T = d['wheelbase'], d['track_width'], d['wheel_radius'], d['wheel_thickness']
    result = []
    # Coordinates below are local to chassis_link: rear axle x=-wb/2, wheel z=-H/2.
    def add(name, shape, material, group='chassis', note=''):
        result.append(Part(name, group, shape, material, note))

    # Narrow nose leaves both front steering assemblies exposed.
    outline = [(-L/2+15,-W/2), (wb/2-52,-W/2), (L/2-12,-50),
               (L/2, -35), (L/2,35), (L/2-12,50),
               (wb/2-52,W/2), (-L/2+15,W/2), (-L/2,W/2-15), (-L/2,-W/2+15)]
    floor = cq.Workplane('XY', origin=(0,0,-H/2+8)).polyline(outline).close().extrude(4).val()
    hole_xy = [(x,y) for x in (-wb/2,-50,40,wb/2-40) for y in (-70,70)]
    for x,y in hole_xy:
        floor = floor.cut(cylinder(1.7,8,(x,y,-H/2+6)))
    for x in (-35,25,85):
        floor = floor.cut(box(25,45,8,(x,0,-H/2+10)))
    add('lower_deck_4mm', floor, 'aluminium', note='M3 clearance holes 3.4 mm; lightening windows')
    for sign in (-1,1):
        y=sign*82
        rail=box(wb+45,12,16,(-12,y,-H/2+20)).cut(box(wb+47,8,12,(-12,y,-H/2+20)))
        add(f'frame_rail_{sign}',rail,'frame')
    for x,label in [(-wb/2,'rear'),(wb/2,'front')]:
        add(f'{label}_crossmember',box(18,track-46,12,(x,0,-H/2+14)),'blue')
    for x in (-95,65):
        for y in (-70,70):
            add(f'standoff_{x}_{y}',ring(4,1.7,49,(x,y,-H/2+12)),'gold')
            add(f'deck_screw_{x}_{y}',cylinder(3,2,(x,y,-H/2+65)),'aluminium')
    upper = box(200,164,3,(-15,0,8))
    for x in (-95,65):
        for y in (-70,70):
            upper=upper.cut(cylinder(1.7,6,(x,y,5)))
    upper=upper.cut(box(95,65,7,(-25,0,8)))
    add('upper_equipment_deck',upper,'blue')
    # Battery sits inside the frame, visible through the upper-deck window.
    add('battery_pack',box(115,48,30,(-20,0,-20)),'battery',note='Generic envelope, not a selected cell pack')
    for x in (-57,17):
        strap=box(10,54,34,(x,0,-20)).cut(box(12,49,30,(x,0,-20)))
        add(f'battery_strap_{x}',strap,'rubber')
    board=box(65,50,2,(20,48,13))
    for x in (-8,48):
        for y in (28,68):
            board=board.cut(cylinder(1.5,5,(x,y,11)))
    add('controller_pcb',board,'green')
    add('compute_heatsink',box(28,26,5,(20,48,17)),'aluminium')
    for x in range(9,34,4):
        add(f'heatsink_fin_{x}',box(1,26,6,(x,48,22)),'frame')
    add('motor_driver',box(45,30,12,(-60,-48,16)),'green')
    for x in (-72,-60,-48):
        add(f'driver_terminal_{x}',box(7,8,7,(x,-61,24)),'blue')
    add('power_button_base',cylinder(10,6,(-65,45,11)),'frame')
    add('power_button_cap',cylinder(8,7,(-65,45,17)),'red')
    # Two rear gearmotors match the independent rear-wheel velocity interfaces.
    for sign,label in [(1,'left'),(-1,'right')]:
        x=-wb/2; y=sign*(track/2-T/2-43)
        start=(x,y-22,-H/2)
        add(f'rear_{label}_gearmotor',cylinder(15,44,start,(0,1,0)),'aluminium',note='Generic gearmotor envelope')
        add(f'rear_{label}_gearbox',box(31,20,31,(x,sign*(track/2-T/2-11),-H/2)),'frame')
        bracket=box(40,4,37,(x,sign*72,-H/2+3)).cut(cylinder(9,8,(x,sign*72-4,-H/2),(0,1,0)))
        add(f'rear_{label}_motor_bracket',bracket,'blue')
        add(f'rear_{label}_shaft',cylinder(3,26,(x,sign*(track/2-T/2)-13,-H/2),(0,1,0)),'gold')
    for x,label in [(-L/2+5,'rear'),(L/2-9,'front')]:
        add(f'{label}_bumper',box(14,90,16,(x,0,-30)),'rubber')
    for sign in (-1,1):
        add(f'front_lamp_{sign}',box(5,16,10,(L/2-1,sign*24,-20)),'white')
        add(f'rear_lamp_{sign}',box(4,16,8,(-L/2-1,sign*24,-20)),'red')
    # Arrow cutout on the nose makes the forward direction clear.
    arrow=cq.Workplane('XY',origin=(0,0,-H/2+13)).polyline(
        [(wb/2+13,-7),(wb/2+35,-7),(wb/2+35,-16),(wb/2+54,0),
         (wb/2+35,16),(wb/2+35,7),(wb/2+13,7)]).close().extrude(1).val()
    add('front_direction_arrow',arrow,'gold')
    # Steering module: fixed servo case + C-bracket, rotating knuckle + horn.
    for sign,label in [(1,'left'),(-1,'right')]:
        x=wb/2; y=sign*track/2; z=-H/2
        add(f'{label}_steering_bridge',box(30,50,5,(x,sign*(track/2-25),z+76)),'blue')
        add(f'{label}_servo_case',box(40,20,31,(x-10,y,z+97)),'frame',note='Direct kingpin actuation envelope')
        add(f'{label}_servo_cap',box(40,20,3,(x-10,y,z+114)),'blue')
        for dx in (-25,15):
            add(f'{label}_servo_mount_{dx}',box(9,30,3,(x+dx,y,z+80)),'aluminium')
            for dy in (-12,12):
                add(f'{label}_servo_screw_{dx}_{dy}',cylinder(2.5,2,(x+dx,y+dy,z+82)),'gold')
        # Bearing support reaches the kingpin above the tire, leaving the tread free.
        inward=-sign*45
        fork=box(18,12,62,(x,y+inward,z+46)).fuse(
            box(18,55,5,(x,y+inward/2,z+75)))
        fork=fork.cut(cylinder(3.2,10,(x,y,z+70)))
        add(f'{label}_kingpin_fork',fork,'aluminium')
        add(f'{label}_kingpin_bearing',ring(7,3,4,(x,y,z+72)),'gold')
        group=f'steer_{label}'
        add(f'{label}_kingpin',cylinder(3,24,(0,0,59)),'gold',group)
        # C-shaped upright: vertical leg on inboard face, bridge over the tire.
        knuckle=box(12,10,62,(0,inward,29)).fuse(
            box(12,abs(inward)+10,5,(0,inward/2,60)))
        knuckle=knuckle.cut(cylinder(3.1,10,(0,0,55)))
        add(f'{label}_steering_knuckle',knuckle,'blue',group)
        horn=ring(9,3.2,3,(0,0,65)).fuse(box(24,7,3,(-10,0,66.5)))
        horn=horn.cut(cylinder(1.6,6,(-20,0,64)))
        add(f'{label}_servo_horn',horn,'aluminium',group)
        add(f'{label}_stub_axle',cylinder(4,abs(inward)+5,
            (0,min(inward,0)-2,0),(0,1,0)),'aluminium',group)
    # Wheel geometry is shared by all four wheel links; axis is local +Y.
    tire=ring(R,R*.58,T,(0,-T/2,0),(0,1,0))
    # Circumferential grooves plus shallow transverse tread notches.
    for yy in (-T*.25,0,T*.25):
        tire=tire.cut(ring(R+1,R-1.4,1.7,(0,yy-.85,0),(0,1,0)))
    for k in range(24):
        notch=box(2.6,T+2,3,(0,0,R-0.8)).rotate((0,0,0),(0,1,0),k*15)
        tire=tire.cut(notch)
    add('tire_tread',tire,'rubber','wheel')
    rim=ring(R*.58,9,T-5,(0,-(T-5)/2,0),(0,1,0))
    for k in range(6):
        a=2*pi*k/6
        rim=rim.cut(cylinder(6,T,(20*cos(a),-T/2,20*sin(a)),(0,1,0)))
    add('six_spoke_rim',rim,'aluminium','wheel')
    add('wheel_hub',ring(10,4,13,(0,-6.5,0),(0,1,0)),'blue','wheel')
    for side in (-1,1):
        add(f'hub_nut_{side}',ring(6,3,3,(0,side*(T/2-5)-1.5,0),(0,1,0)),'gold','wheel')
    return result


def mounts(dimensions):
    wb=dimensions['wheelbase']*1000; track=dimensions['track_width']*1000
    R=dimensions['wheel_radius']*1000; H=dimensions['body_height']*1000
    return {'chassis':(wb/2,0,R+H/2),
            'steer_left':(wb,track/2,R),'steer_right':(wb,-track/2,R)}


def assembly(dimensions, components):
    model=cq.Assembly(name='ackermann_mechanical_2wd')
    origins=mounts(dimensions)
    for p in components:
        targets=[(p.name,origins[p.group])] if p.group!='wheel' else [
            (f'{axle}_{side}_{p.name}',(x,y,dimensions['wheel_radius']*1000))
            for axle,x in [('rear',0),('front',dimensions['wheelbase']*1000)]
            for side,y in [('left',dimensions['track_width']*500),('right',-dimensions['track_width']*500)]]
        for name,position in targets:
            model.add(p.shape,name=name,loc=cq.Location(cq.Vector(*position)),color=cq.Color(*COLORS[p.material]))
    return model
