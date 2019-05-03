## P-A Leziart, 16 April 2019
# Goal: Visualization of a 3D point cloud with Matplotlib
# Points are previously written in a textfile
# Each row is x,y,z separated by commas

import numpy as np
from matplotlib import pyplot as plt
import glob
    
from mpl_toolkits.mplot3d import Axes3D
from scipy.signal import find_peaks

# Read data from text file
name = glob.glob("/home/leziart/Downloads/data_pro*.txt")
name = name[0]
entry = np.loadtxt(open(name, "rb"), delimiter=",")

# Remove NaN entries
entry = entry[~np.isnan(entry).any(axis=1)]

# Select only one row out of 10 to reduce the number of points
indexes = np.arange(0, entry.shape[0], 1)
indexes = (indexes%10)==0

# Select only the row corresponding to the upper portion of the body
y_limit = int(640 * 0.2) 
y_bool = (entry[:,1] < y_limit)

# Apply the two criterion (1 out of 10 and upper portion of body)
entry = entry[indexes & y_bool,:]

# Compute histogram of depth
hist, bin_edges  = np.histogram(entry[:,2], bins=30)

# Create figure and get axes
fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')

# Plot 3D point cloud with a colormap along depth axis
ax.scatter( entry[:,0], entry[:,1], entry[:,2], c= entry[:,2], cmap="gnuplot")

# Set labels
ax.set_xlabel('X Label')
ax.set_ylabel('Y Label')
ax.set_zlabel('Z Label')

# Display result
plt.show()

# Create another figure to plot the histogram
fig = plt.figure()
ax = fig.add_subplot(111)
ax.hist(entry[:,2], bins=30)
plt.show()

# Create another figure to plot peak detection with scipy
plt.figure()
x = hist
peaks, properties = find_peaks(x, prominence=1)
plt.plot(x)
plt.plot(peaks, x[peaks], "x")
plt.vlines(x=peaks, ymin=x[peaks] - properties["prominences"], ymax = x[peaks], color = "C1")
#plt.hlines(y=properties["width_heights"], xmin=properties["left_ips"], xmax=properties["right_ips"], color = "C1")
plt.show()


    
    
