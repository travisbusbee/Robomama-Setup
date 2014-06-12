from mecode import G
import numpy as np
import pickle
from slidedata import *
from aligndata import *
outfile = r"C:\Users\Lewis Group\Documents\GitHub\Robomama-Setup\cell_printing_out.pgm"

g = G(
    direct_write=False,
    outfile=outfile,
    header=None,
    footer=None,
    print_lines=False,
    )
    

# Cell allignment lines on 22mm slides
data = open("slide_data.txt", "r")
align_data = open("align_data.txt", "r")

z_ref = -83.882370
x_dist_from_left = 11
y_dist_from_bot = 11


surface_array = pickle.load(data)
alignment_array = pickle.load(align_data)
zA, Ax_offset, Ay_offset, zB, Bx_offset, By_offset, zC, Cx_offset, Cy_offset, zD, Dx_offset, Dy_offset, substrate_left_x, substrate_bottom_y = alignment_array.profiles

center_x = substrate_left_x + Ax_offset + x_dist_from_left
center_y = substrate_bottom_y + Ay_offset + y_dist_from_bot
column_spacing = 30
rowspacing = -30

array_centerpoints = ((center_x + column_spacing*0, center_y + rowspacing *0), (center_x + column_spacing*1, center_y + rowspacing *0), (center_x + column_spacing*2, center_y + rowspacing *0),(center_x + column_spacing*3, center_y + rowspacing *0), (center_x + column_spacing*4, center_y + rowspacing *0),
                        (center_x + column_spacing*0, center_y + rowspacing *1), (center_x + column_spacing*1, center_y + rowspacing *1), (center_x + column_spacing*2, center_y + rowspacing *1),(center_x + column_spacing*3, center_y + rowspacing *1), (center_x + column_spacing*4, center_y + rowspacing *1),
                        (center_x + column_spacing*0, center_y + rowspacing *2), (center_x + column_spacing*1, center_y + rowspacing *2), (center_x + column_spacing*2, center_y + rowspacing *2),(center_x + column_spacing*3, center_y + rowspacing *2), (center_x + column_spacing*4, center_y + rowspacing *2))
#print array_centerpoints
#print surface_array.profiles
#print alignment_array.profiles


#def build_array():
    #building 22mm array
    #start_x = center_x
    #start_y = center_y
    #
    #array_centerpoints = np.zeros((5,3,2))
    #
    #increment = 30
    #
    #     
    #        
    #for i in range(0,5,1):
    #    for j in range(0,3,1):
    #        for k in range(0,2,1):
    #            if k:
    #                array_centerpoints[i, j, k] =+ increment*j +start_x
    #            else:
    #                array_centerpoints[i, j, k] =+ -increment*i +start_y
    #
    #array_centerpoints = np.reshape(array_centerpoints,(15,2))
    #
    #print array_centerpoints
    
    
    #Somehow get surface center points
    #surface_array = (0.01, 0.01, 0.01, 0.01, 0.01, 0.01, 0.01, 0.01, 0.01, 0.01, 0.01, 0.01, 0.01, 0.01, 0.01, 0.01)
    

#Settings

pressure_box = 4

cantilever_width = 3.5
cantilever_length = 4.5


base_height=(0.02,)*15#(0.01, 0.01, 0.01, 0.01, 0.01, 0.01, 0.01, 0.01,
             #0.01, 0.01, 0.01, 0.01, 0.01, 0.01, 0.01, 0.01)

base_pressure=(4.8,)*15#(5.5, 5.6, 5.7, 5.8, 5.9, 6.0, 6.1, 6.2)*2#(6.2,)*8+(6.1,)*8#
               
base_speed=((4,)*15)

base_spacing = 0.4


cell_groove_height=(0.025,)*15

cell_groove_pressure=(4.8,)*15
               
cell_groove_speed=(4,)*15

cell_groove_spacing=(0.040, 0.050, 0.060, 0.070, 0.080, 0.040, 0.050, 0.060, 0.070, 0.080, 0.040, 0.050, 0.060, 0.070, 0.080)


def meander_notail(x, y, z, spacing, orientation, tail, speed, clip_direction, nozzle, valve, dwell = 0.5):
    g.feed(30)
    g.abs_move(**{nozzle:z})
    g.feed(1)
    if valve is not None:
        g.set_valve(num = valve, value = 1)
    g.dwell(dwell)
    g.feed(speed)
    g.meander(x=x, y=y, spacing=spacing, orientation='y', tail = False)
    if valve is not None:
        g.set_valve(num = valve, value = 0) 
    g.dwell(0.3)
    
                                                                                                
def meander_tail(x, y, z, spacing, orientation, tail, speed, clip_direction, nozzle, valve, dwell = 0.5):
    g.feed(30)
    g.move(x=-tail)
    g.abs_move(**{nozzle:z})
    g.feed(1) 
    if valve is not None:
        g.set_valve(num = valve, value = 1)
    g.dwell(dwell)
    g.move(x=tail)
    g.feed(speed)
    g.meander(x=x, y=y, spacing=spacing, orientation='y', tail = False)
    g.move(x=tail)
    if valve is not None:
        g.set_valve(num = valve, value = 0) 
    g.dwell(0.3)
    g.clip(axis=nozzle, direction=clip_direction, height=5)
    

#def print_pnipaam_all_22mm_slides(x, y, z, spacing, tail, speed, nozzle, valve, dwell):
#    for i in range(0,15):
#        g.feed(30)
#        g.abs_move(*array_centerpoints[i])
#        meander_tail(x, y, z, spacing, orientation, tail, speed, clip_direction, nozzle, valve, dwell = 0.5)
#
def nozzle_change(nozzles = 'ab'):
    g.feed(40)
    #g.home()
    g.dwell(0.25)
    g.write(';----------nozzle change------------')
    if nozzles=='ab':
        g.abs_move(A=50)
        g.move(x=(Bx_offset - Ax_offset), y = (By_offset - Ay_offset))
    elif nozzles=='ac':
        g.abs_move(A=50)
        g.move(x=(Cx_offset - Ax_offset), y = (Cy_offset - Ay_offset))    
    elif nozzles=='ad':
        g.abs_move(A=50)
        g.move(x=(Dx_offset - Ax_offset), y = (Dy_offset - Ay_offset))
    elif nozzles=='ba':
        g.abs_move(B=50)
        g.move(x=(Ax_offset - Bx_offset), y = (Ay_offset - By_offset))
    elif nozzles=='bc':
        g.abs_move(B=50)
        g.move(x=(Cx_offset - Bx_offset), y = (Cy_offset - By_offset))
    elif nozzles=='bd':
        g.abs_move(B=50)
        g.move(x=(Dx_offset - Bx_offset), y = (Dy_offset - By_offset))
    elif nozzles=='ca':
        g.abs_move(C=50)
        g.move(x=(Ax_offset - Cx_offset), y = (Ay_offset - Cy_offset))
    elif nozzles=='cb':
        g.abs_move(C=50)
        g.move(x=(Bx_offset - Cx_offset), y = (By_offset - Cy_offset))
    elif nozzles=='cd':
        g.abs_move(C=50)
        g.move(x=(Dx_offset - Cx_offset), y = (Dy_offset - Cy_offset))
    elif nozzles=='da':
        g.abs_move(D=50)
        g.move(x=(Ax_offset - Dx_offset), y = (Ay_offset - Dy_offset))
    elif nozzles=='db':
        g.abs_move(D=50)
        g.move(x=(Bx_offset - Dx_offset), y = (By_offset - Dy_offset))
    elif nozzles=='dc':
        g.abs_move(D=50)
        g.move(x=(Cx_offset - Dx_offset), y = (Cy_offset - Dy_offset))
    else:
        g.write('; ---------- input a real nozzle change input...ya idiot--------')

def base_all_22mm_slides(nozzle, valve):
    
    for i in range(0,7):
        g.feed(30)
        g.abs_move(*array_centerpoints[i])
        g.move(x=-8,y=6)
        g.set_pressure(pressure_box, base_pressure[i])
        meander_tail (x=cantilever_width, y=-cantilever_length, z=(base_height[i]+surface_array.profiles[i]), spacing=base_spacing, orientation = 'y', tail = 1, speed=base_speed[i], clip_direction = '+y', nozzle = nozzle, valve = valve, dwell = 0.75 )
        g.move(x=2,y=-4)
        g.set_pressure(pressure_box, base_pressure[i])
        meander_tail (x=cantilever_width, y=-cantilever_length, z=(base_height[i]+surface_array.profiles[i]), spacing=base_spacing, orientation = 'y', tail = 1, speed=base_speed[i], clip_direction = '+y', nozzle = nozzle, valve = valve, dwell = 0.75 )
        
def cell_groove_all_22mm_slides(nozzle, valve):
    
    for i in range(0,7):
        g.feed(30)
        g.abs_move(*array_centerpoints[i])
        nozzle_change(nozzles = 'ab')
        g.move(x=-8,y=6)
        g.set_pressure(pressure_box, base_pressure[i])
        meander_notail(x=cantilever_width, y=-cantilever_length, z=(cell_groove_height[i]+surface_array.profiles[i]), spacing=cell_groove_spacing[i], orientation= 'y', tail= 1, speed=cell_groove_speed[i], clip_direction = '+y', nozzle = nozzle, valve = valve, dwell = 0.5)
        g.move(x=2,y=-4)
        g.set_pressure(pressure_box, base_pressure[i])
        meander_notail(x=cantilever_width, y=-cantilever_length, z=(cell_groove_height[i]+surface_array.profiles[i]), spacing=(cell_groove_spacing[i]+0.05), orientation= 'y', tail= 1, speed=cell_groove_speed[i], clip_direction = '+y', nozzle = nozzle, valve = valve, dwell = 0.5)
        
def set_home_in_z():
    g.write('POSOFFSET CLEAR X Y U A B C D')
    g.feed(30)
    g.abs_move(A=-2, B=-2, C=-2, D=-2)
    g.set_home(A=(-zA -2), B=(-zB -2), C = (-zD - 2), D=(-zD - 2))
   
def clear_XYhome():
    g.write('POSOFFSET CLEAR X Y U')
    


#lets go
g.feed(40)
set_home_in_z()
base_all_22mm_slides(nozzle= 'A', valve = 0)
#nozzle_change(nozzles = 'ab')
#
cell_groove_all_22mm_slides(nozzle= 'B', valve = 1)








#g.feed(40)
#set_home_in_z()
##g.abs_move(A=50, B=50, C=50, D=50)
#g.abs_move(X=(substrate_left_x + x_dist_from_left + Ax_offset), Y=(substrate_bottom_y+y_dist_from_bot+Ay_offset))
#g.abs_move(A=surface_array.profiles[0])
#g.abs_move(A=50)
#g.abs_move(*array_centerpoints[1])
#nozzle_change(nozzles = 'ab')
#g.abs_move(B=surface_array.profiles[1])
g.teardown()