#!/usr/bin/env python
# coding: utf-8

# In[ ]:


# LINEAR VARIABLE TAPER CUT

# The output of this program is a 1x6 vector ToolPosition = [x,y,x,a,c,mark/jump] where ToolPosition[5] toggles the laser state

# The output is written to "ToolPathList.txt"

# The config file is a list of tunable parameters that influences the toolpath

# 4+1 CNC process, XYZA stages run in pseudo-simultanous motion, C is used for positioning


# In[150]:


# CONFIG FILE
# The config file contains parameter used to calculate the toolpath and run metrics
# Units: mm, mm/sec, degrees
import numpy as np
from mpl_toolkits import mplot3d
import matplotlib.pyplot as plt


# Laser raster parameters
jump_speed        = 2000   
mark_speed        = 100    
xy_spacing        = .05#0.005  
z_spacing         = .5#0.05
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
kerf_angle        = 4
# kerf angle must be >= beam_angle
xy_overshoot      = .25
z_overshoot       = .5
top_width_limit   = round(z_depth*np.sin(np.radians(beam_angle)), precision)
print('top_width_limit =',top_width_limit)
top_width_kerf    = round(z_depth*np.sin(np.radians(kerf_angle)), precision)
print('top_width_kerf =',top_width_kerf)
bottom_width      = top_width_kerf
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


# In[151]:


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

# Define first raster side wall 1 with three points (x,y,z)
p_side1 = np.array([initial_xy[0], initial_xy[1] - top_width_kerf/2, z_dimension])
q_side1 = np.array([final_xy[0]  , final_xy[1]   - top_width_kerf/2, z_dimension])
r_side1 = np.array([initial_xy[0], initial_xy[1] - bottom_width/2 + kerf_shape , z_dimension - z_depth])

# Define second raster side wall 2 with three points (x,y,z)
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
print("Material Loss Estimate =")


# In[152]:


# TOOL POSITION GENERATOR
# Measure runtime
start_time = timeit.default_timer()


# Initialize tool position vector as a list (X, Y, Z, A, C, mark/jump)
# ToolPositon[5]: 0 = jump, 1 = mark
tool_position = np.array([0., 0., 0., 0., 0., 0.])


# Open and overwrite a .txt file with the initial vector
w = open("tool_position_list.txt", "w")
w.write("[x, y, z, a, c]")
w.write("\n")
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

# Define raster plane in XY with 4 corners (this plane will change as we iterate over Z)
raster_plane = np.array([
    [p_side2[0], p_side2[1]],
    [q_side2[0], q_side2[1]],
    [q_side1[0], q_side1[1]],
    [p_side1[0], p_side1[1]]])


# Define aRange, the range of values over which the A stage 'rocks'
# This code only works with straight wall toolpaths. This can later be generalized to curved wall toolpaths if necessary by updating the 
#side2_angle = round(np.degrees(np.arctan(np.absolute((p_side2[1] - r_side2[1]))/np.absolute((p_side2[2] - r_side2[2])))),precision)
side2_angle = round(np.degrees(np.arctan((p_side2[1] - r_side2[1]))/(p_side2[2] - r_side2[2])),precision)
#print(side2_angle)
#side1_angle = round(np.degrees(np.arctan(np.absolute((p_side1[1] - r_side1[1]))/np.absolute((p_side1[2] - r_side1[2])))),precision)
side1_angle = round(np.degrees(np.arctan((p_side1[1] - r_side1[1]))/(p_side1[2] - r_side1[2])),precision)
#print(side1_angle)
a_max = round(side2_angle - .5*beam_angle, precision)
a_min = round(side1_angle + .5*beam_angle, precision)
#print(a_max)
#print(a_min)


# Loop over Y in raster plane
y_range = np.arange(-raster_plane[0,1], -raster_plane[3,1], xy_spacing)
y_range = -y_range
a_range = np.linspace(a_max, a_min, np.size(y_range))

for i in y_range:
    # Loop over X in raster plane
    tool_position[0] = raster_plane[0,0]
    tool_position[1] = i
    # add A rotation from a_range array
    a_range_index    = np.where(y_range == i)
    j = np.asscalar(a_range_index[0])
    tool_position[3] = -a_range[j]
    tool_position[5] = 1 # switch laser to mark
    write(w,tool_position)
    print(tool_position)
    
    tool_position[0] = raster_plane[1,0]
    write(w,tool_position)
    print(tool_position)
    
    tool_position[5] = 0 # switch laser to jump
    write(w,tool_position)
    print(tool_position)
    
# Measure runtime
elapsed = timeit.default_timer() - start_time
print('Code runtime   =', round(elapsed,2),'s')
print('Mantis runtime =')
print('Laser uptime   = ')


# In[ ]:


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


# In[ ]:


# bit flipping function
import numpy as np
def bitFlip(bit):
    bit = np.absolute(bit-1)
    return bit
bit = 1
bitFlip(bit)

#print(np.degrees( ))
#print(np.radians( ))
print(15*np.sin(np.radians(3)))


# In[ ]:


'''# 3D Model
# This visualization code encounters numerical bugs for small kerf angles. It's not critical so fix it later. 

# Calculate planar equations for the side walls of thee raster from the config file information
def plane_eq(p,q,r):  
    a1 = q[0] - p[0]
    b1 = q[1] - p[1]
    c1 = q[2] - p[2]
    a2 = r[0] - p[0]
    b2 = r[1] - p[1]
    c2 = r[2] - p[2]
    a  = b1*c2 - b2*c1 
    b  = a2*c1 - a1*c2 
    c  = a1*b2 - b1*a2 
    d  = (-a*p[0] - b*p[1] - c*p[2]) 
    plane = [a, b, c, d]
    z_val = c
    # normalize to plane(2)
    i = 0
    for i in range(0,3):
        plane[i] = -1*plane[i]/z_val
    return plane

# Define first raster side wall 1
p1 = (initial_xy[0], initial_xy[1] - top_width_kerf/2, z_depth)
q1 = (final_xy[0]  , final_xy[1] - top_width_kerf/2  , z_depth)
r1 = (initial_xy[0], initial_xy[1] - bottom_width/2 + kerf_shape ,0)
plane0 = plane_eq(p1, q1, r1)
plane0 = np.around(plane0, precision)
print('p1,q1,r1 =', p1, r1, q1)
print('plane0 =',plane0)
print()

# Define second raster side wall 2
p2 = (initial_xy[0], initial_xy[1] + top_width_kerf/2, z_depth)
q2 = (final_xy[0]  , final_xy[1] + top_width_kerf/2  , z_depth)
r2 = (initial_xy[0], initial_xy[1] + bottom_width/2 + kerf_shape ,0)
plane1 = plane_eq(p2, q2, r2)
plane1 = np.around(plane1, precision)
print('p2,q2,r2 =', p2, r2, q2)
print('plane1 =',plane1)

# Generate an interactive graph of the block with the toolpath illustrated
# range/resolution of graphic
x = np.linspace(-x_dimension/2, x_dimension/2, 100)
y = np.linspace(-y_dimension/2, y_dimension/2, 100)
xx,yy = np.meshgrid(x,y)

Z1 = plane0[0]*xx + plane0[1]*yy + plane0[3]
Z2 = plane1[0]*xx + plane1[1]*yy + plane1[3]

# chop off the parts of the planes that extend beyond the block
chop  = 5
Z1[Z1 < -2] = np.nan
#Z1[Z1 < -chop] = np.nan
Z1[Z1 > chop]  = np.nan

Z2[Z2 < -2] = np.nan
#Z2[Z2 < -chop] = np.nan
Z2[Z2 > chop]  = np.nan

fig = plt.figure()
ax  = fig.gca(projection = '3d')
ax.view_init(elev = 0, azim = 0)

ax.set_xlabel('x')
ax.set_ylabel('y')
ax.set_zlabel('z')

ax.set_xlim(-x_dimension/2-5, x_dimension/2+5)
ax.set_ylim(-y_dimension/2-5, y_dimension/2+5)
ax.set_zlim(0,z_dimension)
surface1 = ax.plot_surface(xx, yy, Z1)
surface2 = ax.plot_surface(xx, yy, Z2)
'''
