## P-A Leziart, 29 April 2019
# Goal: Check the accuracy of the realsense camera to reconstruct the trajectory
# of a person moving in its field of view

from matplotlib import pyplot as plt
import numpy as np
import glob
from math import cos, sin

# Detect text file
names = glob.glob("/home/leziart/Documents/Reconstruct_Trajectory/data*.txt")
names.sort()

# Load text file
data = np.loadtxt(open(names[1], "rb"), delimiter=",")

# Remove NaN entries
data = data[~np.isnan(data).any(axis=1)]

# Get X, Y, Z
x = data[:,0]
y = data[:,1]
z = data[:,2]

# Filter unwanted detections
y = y[(x>-1.1)&(x<2)]
z = z[(x>-1.1)&(x<2)]
x = x[(x>-1.1)&(x<2)]

x = x[7:500]
y = y[7:500]
z = z[7:500]

# Fix offset
x = x - 0.11


# Add colors
colors = range(len(x))

# Plot
fig = plt.figure()
ax = fig.gca()
c = ax.scatter(x, z, c=colors, cmap='gnuplot')
#cbar = fig.colorbar(c)
x_t = [0, 0, -1, 0.55, 0, 0, -1, 0.55, 0, 0]
y_t = [2, 3.37, 3.37, 3.37, 3.37, 4.9, 4.9, 4.9, 4.9, 2]

stack = np.vstack((x_t,y_t))
deg = -3 * 3.1415 / 180
rot = np.array([[cos(deg), -sin(deg)],[sin(deg), cos(deg)]])
stack_rot = np.dot(rot, stack)

plt.plot(stack_rot[0,:]-0.11, stack_rot[1,:], color="g", linewidth=3, linestyle="-")

plt.xlabel("Position along X [m]")
plt.ylabel("Position along Z [m]")
plt.title("Reconstruction of a person's trajectory in the field of view of the camera [m]")
plt.show()