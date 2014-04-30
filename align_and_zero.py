from mecode.devices.keyence_profilometer import KeyenceProfilometer
from mecode import G
kp = KeyenceProfilometer('COM<X>') 
km = KeyenceMicrometer('COM<x>')
g = G(direct_write=True)
value = kp.read()
Substrate_Guess = (200, 200)

def alignRead():
    x=5

def axis_status(axis):

def find_profilometer_center(nozzle, zStart, step, dwell, speed):
    g.feed(15)
    g.abs_move(**{nozzle:zStart})
    g.feed(speed)
    g.dwell(dwell)
    value = kp.read()
    while value is None:
        g.move(**{nozzle:-step})
        g.dwell(dwell)
        value = kp.read()
    g.move(**{nozzle:-value})
    value = kp.read()
    profilometer_middle = g.write('AXISSTATUS(A, DATAITEM_PositionFeedback)')
    
def edge_find(xStart, yStart, zStart, step1, step2, backstep, dwell, speed1, speed2, tolerance, direction='+x'):
    g.feed(20)
    g.abs_move(x=xStart, y=yStart)
    find_profilometer_center(nozzle = 'A', zStart = -70, step = 1, dwell = 0.1, speed = 5)
    xstep1=0
    xstep2=0
    ystep1=0
    ystep2=0
    xbackstep=0
    ybackstep=0
    if direction == '+x':
        xstep1=step1
        xstep2=step2
        xbackstep=-backstep
    elif direction =='-x':
        xstep1=-step1
        xstep2=-step2
        xbackstep=backstep
    elif direction =='+y':
        ystep1=step1
        ystep2=step2 
        ybackstep=-backstep
    elif direction =='-y':
        ystep1=-step1
        ystep2=-step2 
        ybackstep=backstep
    else:
        raise RuntimeError('invalid direction: {}'.format(direction))
    value = kp.read()
    while abs(value) < tolerance:
        g.move(x=xstep1, y=ystep1)
        g.dwell(dwell)
        value = kp.read()
        if value is None:
            value = tolerance + 1
    g.move(x=xbackstep, y=ybackstep)
    while abs(value) < tolerance:
        g.move(x=xstep2, y=ystep2)
        g.dwell(dwell)
        value = kp.read()
        if value is None:
            value = tolerance + 1
    x_coordinate = g.write('AXISSTATUS(X, DATAITEM_PositionFeedback)')
    y_coordinate = g.write('AXISSTATUS(Y, DATAITEM_PositionFeedback)')

def get_xyz_offset(axis, x, y, zStart, floor, speed_fast, speed_slow, zStep1, zStep2, backstep, downstep, dwell, sweep_range, sweep_speed):
    g.set_valve(num = 7, value = 0)
    g.feed(30)
    #Initialize communication with keyence micrometer
    g.abs_move(x=x, y=y)
    g.abs_move(**{axis:zStart})
    g.feed(speed_fast)
    while value is None:
        g.move(**{axis:-zStep1})
        g.dwell(dwell)
        value = km.read()
        pos=g.get_axis_pos(axis=axis)
        if (pos-zStep1)<floor:
            value = None
            raise RuntimeError('next step will break through the set floor')
    g.move(**{nozzle:backstep})   
    g.feed(speed_slow)  
    while value is None:
        g.move(**{axis:-zStep2})
        g.dwell(dwell)
        value = km.read()
        pos=g.get_axis_pos(axis=axis)
        if (pos-zStep1)<floor:
            value = None
            raise RuntimeError('next step will break through the set floor')
    g.move(**{axis:-downstep})
    g.dwell(0.75)    
    (x_offset, y_offset) = km.get_XY()
    g.set_valve(num = 7, value = 1)
    km.send('PW,4')
    g.move(x=-x_offset, y=-y_offset)
    z_axis_position = g.get_axis_pos(axis=axis)
    g.dwell(3)
    dist = ((sweep_range)/1.414213)
    g.feed(10)
    g.move(x=-(dist/2), y=(dist/2))
    g.feed(sweep_speed)
    km.start_z_min()
    g.move(x=dist, y=-dist)
    z_min = km.stop_z_min()
    while z_min < 0.1:
        g.feed(15)
        g.move(x=-dist, y=dist)
        g.feed((sweep_speed/2))
        km.start_z_min()
        g.move(x=dist, y=-dist)
        z_min = km.stop_z_min()
    return (x,x_offset), (y, y_offset), (z_axis_position, z_min)    
    #to get out (x,x_offset), (y, y_offset), (z_axis_position, z_min)=get_xyz_offset()
    
    