from mecode.devices.keyence_profilometer import KeyenceProfilometer
from mecode.devices.keyence_micrometer import KeyenceMicrometer
from mecode.devices.efd_pressure_box import EFDPressureBox
from mecode import G
from mecode.utils import profile_surface , write_cal_file
import pickle
from slidedata import *
f = open("slide_data.txt", "w")
kp = KeyenceProfilometer('COM3') 
km = KeyenceMicrometer('COM8')
pb = EFDPressureBox('COM4')
g = G(direct_write=True, print_lines=False)
#value = kp.read()
Substrate_Guess = (36.05, 124.01)#(200, 200)
A_alignment_save = 'C:\Users\Lewis Group\Desktop\python alignment backup\axis_A.txt'
B_alignment_save = 'C:\Users\Lewis Group\Desktop\python alignment backup\axis_B.txt'
C_alignment_save = 'C:\Users\Lewis Group\Desktop\python alignment backup\axis_C.txt'
D_alignment_save = 'C:\Users\Lewis Group\Desktop\python alignment backup\axis_D.txt'
profile_many_substrates = 'C:\Users\Lewis Group\Desktop\python alignment backup\substrate_rel_positions.txt'
value = kp.read()
print value


def find_profilometer_center(kp, axis, zStart, step, dwell, speed, floor):
    g.feed(15)
    g.abs_move(**{axis:zStart})
    g.feed(speed)
    g.dwell(dwell)
    value = None
    while value is None:
        g.move(**{axis:-step})
        g.dwell(dwell)
        value = kp.read()
        pos = g.get_axis_pos(axis=axis)
        if abs(pos - step)>abs(floor):
            raise RuntimeError('Profilometer about to exceed set floor')
    g.dwell(1.5)
    value = kp.read()
    g.move(**{axis:value})
    g.dwell(1.5)
    value = kp.read()
    profilometer_middle = g.get_axis_pos(axis=axis)+value
    return profilometer_middle
    
def get_z_ref(kp, x, y, zStart,  axis = 'D', find_mid = True):
    g.feed(40)
    g.abs_move(x=x, y=y)
    g.feed(10)
    g.abs_move(**{axis:zStart})
    profilometer_middle = zStart
    if find_mid == True:
        profilometer_middle = find_profilometer_center(kp, axis = 'D', zStart = zStart, step = 1, dwell = 0.1,  speed = 5, floor = -100)    
    g.abs_move(**{axis:profilometer_middle})
    g.dwell(1.5)
    value = kp.read()
    profilometer_middle = g.get_axis_pos(axis=axis)
    profilometer_ref = profilometer_middle + value
    return (profilometer_ref)   
            
def edge_find(kp, xStart, yStart, zStart, step1, step2, backstep, dwell, speed1, speed2, tolerance,  direction='+x', find_mid = True, axis='D'):
    g.feed(40)
    g.abs_move(x=xStart, y=yStart)
    if find_mid == True:
        profilometer_middle = find_profilometer_center(kp, axis = axis, zStart = zStart, step = 1, dwell = 0.1, speed = 5, floor = -100)
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
    g.dwell(0.75)
    value = kp.read()
    if value is None:
        g.move(x=back_step, y=backstep)
        g.dwell(0.5)
        value = kp.read()
    while abs(value) < tolerance:
        g.move(x=xstep2, y=ystep2)
        g.dwell(dwell)
        value = kp.read()
        if value is None:
            value = tolerance + 1
    coordinate = g.get_axis_pos(axis=direction[1])
    return coordinate

def get_xyz_offset(km, axis, x, y, zStart, floor, speed_fast, speed_slow, zStep1, zStep2, backstep, downstep, dwell, sweep_range, sweep_speed):
    g.set_valve(num = 7, value = 0)
    g.feed(40)
    km.get_xy() 
    #Initialize communication with keyence micrometer
    g.abs_move(x=x, y=y)
    g.abs_move(**{axis:zStart})
    g.feed(speed_fast)
    value = km.read(1)
    while value is None:
        g.move(**{axis:-zStep1})
        g.dwell(dwell)
        value = km.read()
        pos=g.get_axis_pos(axis=axis)
        if (pos-zStep1)<floor:
            raise RuntimeError('next step will break through the set floor')
    g.move(**{axis:backstep})   
    g.feed(speed_slow)  
    value = km.read(1)
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
    
    (x_offset, y_offset) = km.read('both')
    g.set_valve(num = 7, value = 1)
    km.set_program(4)
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
    g.feed(25)
    g.abs_move(**{axis:-0.2})
    g.set_valve(num = 7, value = 0)
    return (x,x_offset), (y, y_offset), (z_axis_position, z_min)    
    #to get out (x,x_offset), (y, y_offset), (z_axis_position, z_min)=get_xyz_offset()

def xy_align_profilometer(kp, axis='D'):
    topY = edge_find(kp, xStart=39.5, yStart=330.5, zStart =-88 , step1 = 0.5 , step2 = 0.05, backstep=1, dwell = 0.2, speed1 = 5, speed2 = 1, tolerance = 0.15, direction='-y', find_mid = True, axis = axis) 
    leftX = edge_find(kp, xStart=73, yStart=319, zStart =-88 , step1 = 0.5 , step2 = 0.05, backstep=1, dwell = 0.2, speed1 = 5, speed2 = 1, tolerance = 0.15, direction='-x', find_mid = False, axis = axis)
    return (float(leftX), float(topY))
    
def identify_substrate_location(kp, xStart, yStart, zStart, axis, edges = 'All'):
    g.feed(40)
    g.abs_move(x=xStart, y=yStart)   
    g.feed(20)
    g.abs_move(**{axis:zStart})
    profilometer_middle = find_profilometer_center(kp, axis = axis, zStart = zStart, step = 1, dwell = 0.1,  speed = 5, floor = -100)
    leftX= edge_find(kp, xStart=xStart, yStart=yStart, zStart =profilometer_middle , step1 = 0.4 , step2 = 0.1, backstep=10, dwell = 0, speed1 = 3, speed2 = 1, tolerance = 0.15, direction='-x', find_mid = False)
    bottomY = edge_find(kp, xStart=xStart, yStart=yStart, zStart =profilometer_middle , step1 = 0.4 , step2 = 0.1, backstep=10, dwell = 0, speed1 = 3, speed2 = 1, tolerance = 0.15, direction='-y', find_mid = False)
    #rightX= edge_find(kp, xStart=xStart, yStart=yStart, zStart =profilometer_middle , step1 = 0.75 , step2 = 0.1, backstep=10, dwell = 0, speed1 = 5, speed2 = 1, tolerance = 0.15, direction='+x', find_mid = False)
    #bottomY= edge_find(kp, xStart=xStart, yStart=yStart, zStart =profilometer_middle , step1 = 0.75 , step2 = 0.1, backstep=10, dwell = 0, speed1 = 5, speed2 = 1, tolerance = 0.15, direction='-y', find_mid = False)
    return (leftX, bottomY), (profilometer_middle)




def calculate_xyz_pos(Az_axis_position,  Az_min,  Ax,  Ax_offset,  Ay,  Ay_offset,  Bz_axis_position,  Bz_min,  Bx,  Bx_offset, 
   By,  By_offset, Cz_axis_position, Cz_min, Cx, Cx_offset, Cy, Cy_offset, Dz_axis_position, Dz_min, 
   Dx, Dx_offset, Dy, Dy_offset, profilometer_x_groove_new, profilometer_y_groove_new, ref_prof_position, substrate_z_pos, zSensor_Plate_offset,  prof_ref_cal , 
   Rz_axis_position , Rz_min, Rx_offset, Rx , Ry, Ry_offset, Rx_groove , Ry_groove , profilometer_x_groove_old , profilometer_y_groove_old, reference_axis ):
   print '!!!!!!!!!!Az_axis_position, Az_min, zSensor_Plate_offset, substrate_z_pos, ref_prof_position'
   print Az_axis_position, Az_min, zSensor_Plate_offset, substrate_z_pos, ref_prof_position
   
   zA = Az_axis_position-Az_min-zSensor_Plate_offset + (substrate_z_pos-ref_prof_position)
   zB = Bz_axis_position-Bz_min-zSensor_Plate_offset + (substrate_z_pos-ref_prof_position)
   zC = Cz_axis_position-Cz_min-zSensor_Plate_offset + (substrate_z_pos-ref_prof_position)
   zD = Dz_axis_position-Dz_min-zSensor_Plate_offset + (substrate_z_pos-ref_prof_position)
   
   if reference_axis == 'B':
       zA_rel = zA - zB
       zC_rel = zC - zB
       zD_rel = zD - zB
       zB_rel = 0
   elif reference_axis =='C':
       zA_rel = zA - zC
       zB_rel = zB - zC
       zC_rel = 0
       zD_rel = zD - zC
   elif reference_axis =='D':
       zA_rel = zA - zD
       zB_rel = zB - zD
       zC_rel = zC - zD
       zD_rel = 0
   else:
       zB_rel = zB - zA
       zC_rel = zC - zA
       zD_rel = zD - zA
       zA_rel = 0    
       
   #ref_pin_axis_pos = -43.09999,
   #zReference = ref_pin_axis_pos + (ref_prof_position-prof_ref_cal)+(substrate_z_pos- ref_prof_position)
   #zA = zReference + (Az_axis_position-Rz_axis_position) + (Rz_min - Az_min)                          
   #zB = zReference + (Bz_axis_position-Rz_axis_position) + (Rz_min - Bz_min) 
   #zC = zReference + (Cz_axis_position-Rz_axis_position) + (Rz_min - Cz_min) 
   #zD = zReference + (Dz_axis_position-Rz_axis_position) + (Rz_min - Dz_min) 
   print '!!!!!!!!!!Ry_groove, Ry, Ay, Ry_offset, Ay_offset, profilometer_y_groove_old, profilometer_y_groove_new'
   print Ry_groove, Ry, Ay, Ry_offset, Ay_offset, profilometer_y_groove_old, profilometer_y_groove_new
   if Ax is not None:
       Ax_groove = Rx_groove-(Rx-Ax)+(Rx_offset - Ax_offset)+(profilometer_x_groove_old - profilometer_x_groove_new)
       Ay_groove = Ry_groove-(Ry-Ay)+(Ry_offset - Ay_offset)+(profilometer_y_groove_old - profilometer_y_groove_new)
   else:
       Ax_groove=Ay_groove = None
   if Bx is not None:
       Bx_groove = Rx_groove-(Rx-Bx)+(Rx_offset - Bx_offset)+(profilometer_x_groove_old - profilometer_x_groove_new)
       By_groove = Ry_groove-(Ry-By)+(Ry_offset - By_offset)+(profilometer_y_groove_old - profilometer_y_groove_new)
   else:
       Bx_groove=By_groove = None
   if Cx is not None:
       Cx_groove = Rx_groove-(Rx-Cx)+(Rx_offset - Cx_offset)+(profilometer_x_groove_old - profilometer_x_groove_new)
       Cy_groove = Ry_groove-(Ry-Cy)+(Ry_offset - Cy_offset)+(profilometer_y_groove_old - profilometer_y_groove_new)
   else:
       Cx_groove=Cy_groove = None
   if Dx is not None:
       Dx_groove = Rx_groove-(Rx-Dx)+(Rx_offset - Dx_offset)+(profilometer_x_groove_old - profilometer_x_groove_new)
       Dy_groove = Ry_groove-(Ry-Dy)+(Ry_offset - Dy_offset)+(profilometer_y_groove_old - profilometer_y_groove_new)
   else:
       Dx_groove=Dy_groove = None
 
   return (zA, Ax_groove, Ay_groove), (zB, Bx_groove, By_groove), (zC, Cx_groove, Cy_groove), (zD, Dx_groove, Dy_groove), (zA_rel, zB_rel, zC_rel, zD_rel)

def nozzle_change(Ax_groove, Ay_groove, Bx_groove, By_groove, Cx_groove, Cy_groove, Dx_groove, Dy_groove, profilometer_x_groove_new, profilometer_y_groove_new,nozzles = 'ab'):
    g.feed(40)
    #g.home()
    g.dwell(0.25)
    g.write(';----------nozzle change------------')
    if nozzles=='ab':
        #g.abs_move(A=50)
        g.move(x=Bx_groove - Ax_groove, y=By_groove - Ay_groove)
    elif nozzles=='ac':
        #g.abs_move(A=50)
        g.move(x=Cx_groove - Ax_groove, y=Cy_groove - Ay_groove)   
    elif nozzles=='ad':
        #g.abs_move(A=50)
        g.move(x=Dx_groove - Ax_groove, y=Dy_groove - Ay_groove)
    elif nozzles=='ba':
        #g.abs_move(B=50)
        g.move(x=Ax_groove - Bx_groove, y=Ay_groove - By_groove)
    elif nozzles=='bc':
        #g.abs_move(B=50)
        g.move(x=Cx_groove - Bx_groove, y=Cy_groove - By_groove)
    elif nozzles=='bd':
        #g.abs_move(B=50)
        g.move(x=Dx_groove - Bx_groove, y=Dy_groove - By_groove)
    elif nozzles=='ca':
        #g.abs_move(C=50)
        g.move(x=Ax_groove - Cx_groove, y=Ay_groove - Cy_groove)
    elif nozzles=='cb':
        #g.abs_move(C=50)
        g.move(x=Bx_groove - Cx_groove, y=By_groove - Cy_groove)
    elif nozzles=='cd':
        #g.abs_move(C=50)
        g.move(x=Dx_groove - Cx_groove, y=Dy_groove - Cy_groove)
    elif nozzles=='da':
        #g.abs_move(D=50)
        g.move(x=Ax_groove - Dx_groove, y=Ay_groove - Dy_groove)
    elif nozzles=='db':
        #g.abs_move(D=50)
        g.move(x=Bx_groove - Dx_groove, y=By_groove - Dy_groove)
    elif nozzles=='dc':
        #g.abs_move(D=50)
        g.move(x=Cx_groove - Dx_groove, y=Cy_groove - Dy_groove)
    elif nozzles == 'pa':
        #g.abs_move(D=50)
        g.move(x=Ax_groove - profilometer_x_groove_new, y=Ay_groove - profilometer_y_groove_new)
    elif nozzles == 'pb':
        #g.abs_move(D=50)
        g.move(x=Bx_groove - profilometer_x_groove_new, y=By_groove - profilometer_y_groove_new)
    elif nozzles == 'pc':
        #g.abs_move(D=50)
        g.move(x=Cx_groove - profilometer_x_groove_new, y=Cy_groove - profilometer_y_groove_new)
    elif nozzles == 'pd':
        #g.abs_move(D=50)
        g.move(x=Dx_groove - profilometer_x_groove_new, y=Dy_groove - profilometer_y_groove_new)
    else:
        g.write('; ---------- input a real nozzle change input...ya idiot--------')                                                                              
                                                                                                                                                                                                                                       
def run_alignment_script(Substrate_Guess, align_A, align_B, align_C, align_D,  zSensor_Plate_offset, prof_ref_cal , Rz_axis_position , Rz_min, Rx_offset, Rx , Ry , Ry_offset , Rx_groove , Ry_groove , Substrate_xRef = 37, Substrate_yRef = -26, x_zref = 56.40, y_zref = 284.93 , profilometer_x_groove_old = 70.49999, profilometer_y_groove_old = 327.850075, start_zref = -94, axis = 'D'):
       
   #get_xyz_offset(km, axis, x, y, zStart, floor, speed_fast, speed_slow, zStep1, zStep2, backstep, downstep, dwell, sweep_range, sweep_speed)
   #move to safe home
   g.feed(40)
   #g.abs_move(**{axis:-15})
   g.write('POSOFFSET CLEAR X Y U A B C D')
   g.abs_move(A=-0.5, B=-0.5, C=-0.5, D=-0.5)
   Az_axis_position, Az_min, Ax, Ax_offset, Ay, Ay_offset = -35.4000021246, 2.9093, 586.075, 1.5, 367.82, 1.5
   if align_A is True:
       (Ax,Ax_offset), (Ay, Ay_offset), (Az_axis_position, Az_min) = get_xyz_offset(km, axis='A', x=586.075, y=367.82, zStart= -15, floor = -49.25, speed_fast = 10, speed_slow = 2, zStep1 = 1, zStep2 = 0.1, backstep = 3.5, downstep = 0.4, dwell = 0, sweep_range = 1, sweep_speed = 0.025)
   else:
       (Ax,Ax_offset) = (Ay, Ay_offset) = (Az_axis_position, Az_min) = (0, 0)
   if align_B is True:
       (Bx,Bx_offset), (By, By_offset), (Bz_axis_position, Bz_min) = get_xyz_offset(km, axis='B', x=482.075, y=367.82, zStart= -15, floor = -49.25, speed_fast = 10, speed_slow = 2, zStep1 = 1, zStep2 = 0.1, backstep = 3.5, downstep = 0.4, dwell = 0, sweep_range = 1.5, sweep_speed = 0.1)
   else:
       (Bx,Bx_offset) = (By, By_offset) = (Bz_axis_position, Bz_min) = (0, 0)
   if align_C is True:
       (Cx,Cx_offset), (Cy, Cy_offset), (Cz_axis_position, Cz_min) = get_xyz_offset(km, axis='C', x=378.075, y=367.82, zStart= -15, floor = -49.25, speed_fast = 10, speed_slow = 2, zStep1 = 1, zStep2 = 0.1, backstep = 3.5, downstep = 0.4, dwell = 0, sweep_range = 1.5, sweep_speed = 0.1)
   else:
       (Cx,Cx_offset) = (Cy, Cy_offset) = (Cz_axis_position, Cz_min) = (0, 0)
   if align_D is True:
       (Dx,Dx_offset), (Dy, Dy_offset), (Dz_axis_position, Dz_min) = get_xyz_offset(km, axis='D', x=299.075, y=367.82, zStart= -15, floor = -49.25, speed_fast = 10, speed_slow = 2, zStep1 = 1, zStep2 = 0.1, backstep = 3.5, downstep = 0.4, dwell = 0, sweep_range = 1.5, sweep_speed = 0.1)
   else:
       (Dx,Dx_offset) = (Dy, Dy_offset) = (Dz_axis_position, Dz_min) = (0, 0)
   
  

   g.feed(25)
   g.abs_move(**{axis:-0.5})
   
   # find grooves on alignment plate
   
   (profilometer_x_groove_new, profilometer_y_groove_new) = xy_align_profilometer(kp, axis = axis)
   print '!!!!!!', profilometer_y_groove_new, type(profilometer_y_groove_new)
   
   #x_prof_offset = profilometer_x_groove_new - profilometer_x_groove_old
   #y_prof_offset = profilometer_y_groove_new - profilometer_y_groove_old
   g.feed(25)
   g.abs_move(**{axis:-40})
   
   #Profile z_ref location
   (ref_prof_position) = get_z_ref(kp, x = x_zref , y=y_zref , zStart=start_zref , axis = axis, find_mid = True)
   
   g.feed(25)
   g.abs_move(**{axis:-40})
   
   #find substrate boundaries
   (substrate_left_x, substrate_bottom_y), (prof_substrate_start)=  identify_substrate_location(kp, xStart = Substrate_Guess[0], yStart = Substrate_Guess[1], zStart = -89.9, axis = axis, edges = 'All')
   
   g.feed(25)
   g.abs_move(**{axis:-40})
   
   #move to substrate reference and obtain profilometer ref
   (substrate_z_pos) = get_z_ref(kp, x= substrate_left_x + Substrate_xRef, y=substrate_bottom_y + Substrate_yRef, zStart=prof_substrate_start, axis = axis, find_mid=False)
   #g.feed(25)
   #g.abs_move(**{axis:-40})
   #
   print '!!!!!! right before calc', profilometer_y_groove_new, type(profilometer_y_groove_new)
   #calculate relative axis xyz positions
   (zA, Ax_groove, Ay_groove), (zB, Bx_groove, By_groove), (zC, Cx_groove, Cy_groove), (zD, Dx_groove, Dy_groove), (zA_rel, zB_rel, zC_rel, zD_rel) = calculate_xyz_pos(Az_axis_position,  Az_min,  Ax,  Ax_offset,  Ay,  Ay_offset,  Bz_axis_position,  Bz_min,  Bx,  Bx_offset, 
   By,  By_offset, Cz_axis_position, Cz_min, Cx, Cx_offset, Cy, Cy_offset, Dz_axis_position, Dz_min, 
   Dx, Dx_offset, Dy, Dy_offset, profilometer_x_groove_new, profilometer_y_groove_new, ref_prof_position, substrate_z_pos, zSensor_Plate_offset,  prof_ref_cal , 
   Rz_axis_position , Rz_min, Rx_offset, Rx , Ry, Ry_offset, Rx_groove , Ry_groove , profilometer_x_groove_old , profilometer_y_groove_old, reference_axis = None )
   
   #nozzle_change(Ax_groove, Ay_groove, Bx_groove, By_groove, Cx_groove, Cy_groove, 
            #Dx_groove, Dy_groove, profilometer_x_groove_new, profilometer_y_groove_new, nozzles = 'pa')
   if align_A is True:
       Ax_offset = (Ax_groove - profilometer_x_groove_new)
       Ay_offset = (Ay_groove - profilometer_y_groove_new)
   else:
       Ax_offset = Ay_offset = 0
   if align_B is True:
       Bx_offset = (Bx_groove - profilometer_x_groove_new)
       By_offset = (By_groove - profilometer_y_groove_new)
   else:
       Bx_offset = By_offset = 0
   if align_C is True:
       Cx_offset = (Cx_groove - profilometer_x_groove_new)
       Cy_offset = (Cy_groove - profilometer_y_groove_new)
   else:
       Cx_offset = Cy_offset = 0
   if align_D is True:
       Dx_offset = (Dx_groove - profilometer_x_groove_new)
       Dy_offset = (Dy_groove - profilometer_y_groove_new)
   else:
       Dx_offset = Dy_offset = 0    
   
    
   #do profiling array
   
   #make cal file
   
   #load cal file
   
   # kp.disconnect  - Closes comport
   print '(x_offset), (y_offset): ' ,  Ax_offset, Ay_offset
   print 'substrate_left_x, substrate_bottom_y: ' ,  substrate_left_x, substrate_bottom_y
   #print '(Ax,Ax_offset), (Ay, Ay_offset), (Az_axis_position, Az_min): ' ,  (Ax,Ax_offset), (Ay, Ay_offset), (Az_axis_position, Az_min)
   #print 'prof grooves location: ' , (profilometer_x_groove_new, profilometer_y_groove_new)
   #print 'prof ref z: ' ,  (ref_prof_position)
   #print 'substrate orgin: '(substrate_left_x, substrate_top_y)
   #print 'prof substrate z: ' (substrate_z_pos)
   return Ax_offset, Ay_offset, Bx_offset, By_offset, Cx_offset, Cy_offset, Dx_offset, Dy_offset, substrate_left_x, substrate_bottom_y

  
def profile_coverslips(kp, f, profile_many_substrates, x_dist_from_left, y_dist_from_bot, rows, columns, row_dist, column_dist, dwell):    
    g.feed(30)
    g.abs_move(x=(substrate_left_x + x_dist_from_left), y= (substrate_bottom_y+y_dist_from_bot))
    profilometer_center = find_profilometer_center(kp, axis = 'D', zStart = -80, step = 1, dwell = 0.1,  speed = 5, floor = -100)
    ref_value = kp.read()
    profile_many_substrates = open(profile_many_substrates, 'w+')
    for j in range (rows):
        for i in range((columns-1)):
            g.dwell(dwell)
            value = kp.read()
            dif = value - ref_value
            profile_many_substrates.writelines(dif)
            profile_many_substrates.write('\n')
            g.move(x=column_dist)
        g.dwell(dwell)
        value = kp.read()
        dif = value - ref_value
        profile_many_substrates.writelines(dif)
        profile_many_substrates.write('\n') 
        g.move(x=-(column_dist*(columns-1)), y=-row_dist)
    profile_many_substrates.close()    
              
              
Ax_offset, Ay_offset, Bx_offset, By_offset, Cx_offset, Cy_offset, Dx_offset, Dy_offset, substrate_left_x, substrate_bottom_y =run_alignment_script(Substrate_Guess, align_A=True, align_B=True, align_C=False, align_D=False,  zSensor_Plate_offset = 42.8854, prof_ref_cal = 96.03958, Rz_axis_position = -43.099992493421, Rz_min=+02.82200, Rx_offset=1.7944, Rx = 586.075, Ry = 367.82, Ry_offset = 0.64815, Rx_groove = 400.2418, Ry_groove = 342.4803, Substrate_xRef = 37, Substrate_yRef = 26, x_zref = 54.588, y_zref = 280.322 , profilometer_x_groove_old = 70.4999, profilometer_y_groove_old = 327.850046, start_zref = -94, axis = 'D')         

profile_coverslips(kp, profile_many_substrates, x_dist_from_left = 11, y_dist_from_bot = 11, rows = 3, columns = 5, row_dist = 30, column_dist = 30, dwell = 0.5)
surface = profile_surface(g, kp, x_start=substrate_left_x + 2, x_stop = substrate_left_x + 62, x_step = 10, y_start=substrate_bottom_y + 2, y_stop = substrate_bottom_y + 52, y_step = 10)

#write_cal_file('C:\Users\Lewis Group\Desktop\Calibration\CAL_output.cal', surface, x_start= substrate_left_x + 2, x_stop = substrate_left_x + 62, x_step = 10, y_start = substrate_bottom_y + 2, y_stop = substrate_bottom_y + 52,
#                   y_step = 10, x_offset = Ax_offset, y_offset = Ay_offset, mode='w+', ref_zero=True)
#                   
#write_cal_file('C:\Users\Lewis Group\Desktop\Calibration\CAL_output.cal', surface, x_start= substrate_left_x + 2, x_stop = substrate_left_x + 62, x_step = 10, y_start = substrate_bottom_y + 2, y_stop = substrate_bottom_y + 52,
#                   y_step = 10, x_offset = Bx_offset, y_offset = By_offset, mode='a', ref_zero=True)
#
#write_cal_file('C:\Users\Lewis Group\Desktop\Calibration\CAL_output.cal', surface, x_start= substrate_left_x + 2, x_stop = substrate_left_x + 62, x_step = 10, y_start = substrate_bottom_y + 2, y_stop = substrate_bottom_y + 52,
#                   y_step = 10, x_offset = Cx_offset, y_offset = Cy_offset, mode='a', ref_zero=True)
#
#write_cal_file('C:\Users\Lewis Group\Desktop\Calibration\CAL_output.cal', surface, x_start= substrate_left_x + 2, x_stop = substrate_left_x + 62, x_step = 10, y_start = substrate_bottom_y + 2, y_stop = substrate_bottom_y + 52,
#                   y_step = 10, x_offset = Dx_offset, y_offset = Dy_offset, mode='a', ref_zero=True)


#(substrate_left_x, substrate_top_y), (substrate_right_x, substrate_bottom_y), (prof_substrate_start)=  identify_substrate_location(kp, xStart = 51.20, yStart = 36.46, zStart = -89.9, axis = 'D', edges = 'All')   
#print (substrate_left_x, substrate_top_y)   
#(profilometer_x_groove_new, profilometer_y_groove_new) = xy_align_profilometer(kp, axis = 'D')
#profilometer_center = find_profilometer_center(kp, axis = 'D', zStart = -84, step = 1, dwell = 0.2,  speed = 5, floor = -100)   
#(Ax,Ax_offset), (Ay, Ay_offset), (Az_axis_position, Az_min) = get_xyz_offset(axis='B', x=482.32, y=366.967, zStart= -15, floor = -49.25, speed_fast = 10, speed_slow = 2, zStep1 = 1, zStep2 = 0.1, backstep = 3.5, downstep = 0.4, dwell = 0, sweep_range = 1.5, sweep_speed = 0.25)    
#print (profilometer_x_groove_new, profilometer_y_groove_new)
   
kp.disconnect()   
km.disconnect()
pb.disconnect() 