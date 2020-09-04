#!/usr/bin/env python
# coding: utf-8

# In[1]:


# LINEAR VARIABLE TAPER CUT

# The output of this program is a 1x6 vector tool_position = [x,y,x,a,c,mark/jump] where tool_position[5] toggles the laser state

# The output is written to "tool_position_list.txt"

# The config file is a list of tunable parameters that influences the toolpath

# 4+1 CNC process, XYZA stages run in pseudo-simultanous motion, C is used for positioning


# In[94]:


# CONFIG FILE
# The config file contains parameter used to calculate the toolpath and run metrics
# Units: mm, mm/sec, degrees
import numpy as np
from mpl_toolkits import mplot3d
import matplotlib.pyplot as plt


# Laser raster parameters
jump_speed        = 2000   
mark_speed        = 100    
xy_spacing        = 0.005
z_spacing         = 0.05
beam_angle        = 3
precision         = 4
# the parameters below are not configured yet
#focusDiameter     = .02
#rayleighLength    = .6
#PulseRate         = 5000


# Desired cut line
# Only use cut lines that are parallel to the x-axis until block rotation subroutine is implemented
initial_xy        = np.array([-5,0])
final_xy          = np.array([5,0])
z_depth           = 3.5
kerf_angle        = beam_angle
# kerf angle must be >= beam_angle
xy_overshoot      = .25
z_overshoot       = .5
top_width_min     = round(z_depth*np.sin(np.radians(beam_angle)), precision)
print('top_width_limit =',top_width_limit)
top_width_kerf    = round(z_depth*np.sin(np.radians(kerf_angle)), precision)
print('top_width_kerf =',top_width_kerf)
bottom_width      = 0#top_width_kerf
# bottom_width defines the bottom width of a linear cut
kerf_shape        = 0#top_width_kerf/2
# kerf_shape is a parameter that shears the 3D cut shape
# = 0 the cut is symmetric on both side walls 
# < 0 means that the 3D cut shape shears in + y direction
# > 0 means that the 3D cut shape shears in - y direction
# sample toolpaths:
# we get two parallel side walls forbottom_width = top_width_kerf
# we get two vertical side walls for bottom_width = top_width_kerf, kerf_shape   = 0

    
# Block measurements, assume rectangular prism geometry
z_dimension       = 3.5
x_dimension       = 10
y_dimension       = 10


# Block placement offsets (Alex Chen's CV offset correction code)
# the parameters below are not configured yet 
#origin_x          = 0
#origin_y          = 0
#physical_rotation = 0


# In[95]:


# TOOLPATH VISUALS
import timeit
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from mpl_toolkits.mplot3d.art3d import Poly3DCollection, Line3DCollection
get_ipython().run_line_magic('matplotlib', 'notebook')


# Measure runtime
start_time = timeit.default_timer()


# 2D Model (top down XY view of cutline)
fig_scale = 2

plt.figure(figsize = [x_dimension/fig_scale/2, x_dimension/fig_scale/2])
plt.xlim(-x_dimension/2, x_dimension/2)
plt.ylim(-y_dimension/2, y_dimension/2)
plt.title('Top Down View')
plt.xlabel("X Axis")
plt.ylabel("Y Axis")

x_Range = np.linspace(initial_xy[0], final_xy[0])
y_Range = np.linspace(initial_xy[1], final_xy[1])
plt.plot(x_Range, y_Range)

plt.show()


# 2D Model (side YZ view of cutline)
plt.figure(figsize = [y_dimension/fig_scale, z_dimension/fig_scale])
plt.xlim(-y_dimension/2, y_dimension/2)
plt.ylim(0, z_dimension)
plt.title('Side View')
plt.xlabel("Y Axis")
plt.ylabel("Z Axis")

# Define first raster side1 with three points (x,y,z)
p_side1 = np.array([initial_xy[0], initial_xy[1] - top_width_kerf/2, z_dimension])
q_side1 = np.array([final_xy[0]  , final_xy[1]   - top_width_kerf/2, z_dimension])
r_side1 = np.array([initial_xy[0], initial_xy[1] - bottom_width/2 + kerf_shape , z_dimension - z_depth])

# Define second raster side2 with three points (x,y,z)
p_side2 = np.array([initial_xy[0], initial_xy[1] + top_width_kerf/2, z_dimension])
q_side2 = np.array([final_xy[0]  , final_xy[1]   + top_width_kerf/2, z_dimension])
r_side2 = np.array([initial_xy[0], initial_xy[1] + bottom_width/2 + kerf_shape, z_dimension - z_depth])

y_side1_Range = np.linspace(r_side1[1], p_side1[1])
z_side1_Range = np.linspace(r_side1[2], p_side1[2])

y_side2_Range = np.linspace(r_side2[1], p_side2[1])
z_side2_Range = np.linspace(r_side2[2], p_side2[2])

plt.plot(y_side1_Range, z_side1_Range)
plt.plot(y_side2_Range, z_side2_Range)

plt.show()


# Measure runtime
elapsed = timeit.default_timer() - start_time
#print('time elapsed =', round(elapsed,2),'s')


# In[106]:


# TOOL POSITION GENERATOR
# Measure runtime
start_time = timeit.default_timer()


# Initialize tool position vector as a list (X, Y, Z, A, C, mark/jump)
# ToolPositon[5]: 0 = jump, 1 = mark
tool_position = np.array([0., 0., 0., 0., 0., 0.])


# Open and overwrite a .txt file with the initial vector
w = open("tool_position_list.txt", "w")
#w.write("[x, y, z, a, c]")
#w.write("\n")
w.write(str(tool_position))
w.write("\n")
w.close()


# Define function to append updated position vectors to the .txt file
def write(w, tool_position):
    w = open("tool_position_list.txt", "a")
    w.write(str(tool_position))
    w.write("\n")
    w.close()
    
    
'''   
# Rotate the block in C so that the cut line runs parallel to the X axis
# not yet implemented. Only use raster lines that are parallel to the x-axis

c = np.arctan((final_xy[1]-initial_xy[1])/(final_xy[0]-initial_xy[0]))
c = round(np.degrees(c),precision)
tool_position[4] = c
write(w,tool_position)
print(tool_position)

# transform XY coordinates to match new block rotation using rotation matrix
function   = 1 
initial_xy = function*initial_xy
final_xy   = function*final_xy
'''

# Loop raster planes over Z

# Define raster plane in XY with 4 corners (this plane will change shape as we iterate over Z)
raster_plane = np.array([
    [p_side2[0], p_side2[1]],
    [q_side2[0], q_side2[1]],
    [q_side1[0], q_side1[1]],
    [p_side1[0], p_side1[1]]])

# Define the function that iterates the raster plane shape over Z
# based on the point-slope formula
m_side2  = (r_side2[2] - p_side2[2])/(r_side2[1] - p_side2[1])
m_side1  = (r_side1[2] - p_side1[2])/(r_side1[1] - p_side1[1])

#print(p_side2[1], ',', p_side2[2])

def new_corner(m, z_spacing, x, y):
    y1 = -z_spacing/m + y
    return np.around((x, y1),precision)


# Define z_range, the range over which the cut will raster in Z
z_range = np.around(np.linspace(z_dimension - z_depth, z_dimension, int(round(z_depth/z_spacing, 0))),precision)
z_range = z_range[::-1]

# Generate and loop over raster XY planes
x_i2 = p_side2[0]
y_i2 = p_side2[1]
#print(x_i2, y_i2)
#print(new_corner(m_side2, z_spacing, x_i, y_i))
#print()
x_i1 = p_side1[0]
y_i1 = p_side1[1]
#print(x_i1, y_i1)
#print(new_corner(m_side1, z_spacing, x_i1, y_i1))
#print()



#print(raster_plane[0])
#raster_plane[0] = 

'''def new_raster_plane(m_side2, m_side1, z_spacing, x_i2, x_i1, y_i2, y_i1):

    return np.around(np.array([
    [p_side2[0], p_side2[1]],
    [q_side2[0], q_side2[1]],
    [q_side1[0], q_side1[1]],
    [p_side1[0], p_side1[1]]]),precision)'''



#for i in z_range:
    # generate raster_plane
    

  

# Define a_range, the range of values over which the A stage 'rocks'
# This code only works with straight wall toolpaths. This can later be generalized to curved wall toolpaths if necessary by updating the angle with every iteration.
#side2_angle = round(np.degrees(np.arctan(np.absolute((p_side2[1] - r_side2[1]))/np.absolute((p_side2[2] - r_side2[2])))),precision)
side2_angle = round(np.degrees(np.arctan((p_side2[1] - r_side2[1]))/(p_side2[2] - r_side2[2])),precision)
#print(side2_angle)
#side1_angle = round(np.degrees(np.arctan(np.absolute((p_side1[1] - r_side1[1]))/np.absolute((p_side1[2] - r_side1[2])))),precision)
side1_angle = round(np.degrees(np.arctan((p_side1[1] - r_side1[1]))/(p_side1[2] - r_side1[2])),precision)
#print(side1_angle)
a_max = side2_angle - .5*beam_angle
a_min = side1_angle + .5*beam_angle
# This computation is stable for values of bottom_width and kerf_shape that are within the y bounds of top_width_kerf. This works for OG but may need to be reworked for more complex processes 
#print(a_max)
#print(a_min)

# Loop over y_range in raster plane
y_range = np.arange(-raster_plane[0,1], -raster_plane[3,1], xy_spacing)
y_range = -y_range
a_range = np.around(np.linspace(a_max, a_min, np.size(y_range)), precision)


for i in z_range:
    # generate raster_plane
    tool_position[2] = i
    write(w,tool_position)
    print(tool_position)
    for i in y_range:
        # loop over a single raster XY plane
        tool_position[0] = raster_plane[0,0]
        tool_position[1] = i
        # add A rotation from a_range array
        # do this via vectorization instead...
        a_range_index    = np.where(y_range == i)
        j = np.asscalar(a_range_index[0])
        tool_position[3] = -a_range[j]
        # switch laser to mark
        tool_position[5] = 1 
        write(w,tool_position)
        print(tool_position)
    
        tool_position[0] = raster_plane[1,0]
        write(w,tool_position)
        print(tool_position)
    
        # switch laser to jump
        tool_position[5] = 0 
        write(w,tool_position)
        print(tool_position)

# Measure runtime
elapsed = timeit.default_timer() - start_time
print('Code runtime   =', round(elapsed,2),'s')
print('Mantis runtime =')
print('Laser uptime   =')
print('Material loss  =')

# The code in its form can stably output a zero taper rocking cut for a 3 degree beam/kerf angle. Now convert it to a form that can run on the Labview app
# Issues to fix: raster_plane function, 3d graphic, raster pattern 'snake-like', robust to sign/numerical errors, c rotation, 90deg A rotation for OG cut, runtime metrics, organize/clean code, rename raster_plane to layer or sth, change all np.arange to np.linspace


# In[108]:


# Convert to code that can run on the DF Laser.vi app


# In[65]:


# METRICS
# Measure runtime (copy and paste code into cells)
import timeit
start_time = timeit.default_timer()
# code
elapsed = timeit.default_timer() - start_time
print('time elapsed =', round(elapsed,2),'s')


# Toolpath runtime estimate
# create a running time counter that propagates through the code


# Laser uptime estimate


# Material loss estimate


# Stage Kinematics

