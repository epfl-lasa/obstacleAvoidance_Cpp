## P-A Leziart, 16 April 2019
# Goal: Visualization of a 3D point cloud with Matplotlib
# Points are previously written in a textfile
# Each row is x,y,z separated by commas

import numpy as np
from matplotlib import pyplot as plt
import glob
    
from mpl_toolkits.mplot3d import Axes3D
from scipy.signal import find_peaks

def make_square_axes(ax):
    """Make an axes square in screen units."""
    ax.set_aspect(1 / ax.get_data_ratio())
    
# Read data from text file
name = glob.glob("/home/leziart/Downloads/data_pro*.txt")
name = glob.glob("/home/leziart/Documents/ProcessDepthImgFiles/data_process_depth_img_*.txt")
name = name[0]
entry = np.loadtxt(open(name, "rb"), delimiter=",")

# Remove NaN entries
entry = entry[~np.isnan(entry).any(axis=1)]

# Select only one row out of 10 to reduce the number of points
indexes = np.arange(0, entry.shape[0], 1)
indexes = (indexes%10)==0

# Select only the row corresponding to the upper portion of the body
y_limit = int(640 * 0.25) 
y_bool = (entry[:,1] < y_limit)
z_bool = (entry[:,4] < (min(entry[:,4])+0.25*(max(entry[:,4])-min(entry[:,4]))))
# Apply the two criterion (1 out of 10 and upper portion of body)
#entry = entry[indexes & y_bool,:]
#entry = entry[y_bool,:]

# Compute histogram of depth
hist, bin_edges  = np.histogram(entry[:,2], bins=30)

# Create 2D figure
fig = plt.figure()
ax = fig.add_subplot(111)
c = ax.scatter( entry[:,0], entry[:,1], c= entry[:,4], cmap="gnuplot", vmin=2, vmax=max(entry[:,4]))
ax.set_aspect("equal")
ax.set_xlabel('Position in the picture along X [pixel]')
ax.set_ylabel('Position in the picture along Y [pixel]')
ax.invert_yaxis()
cbar = fig.colorbar(c)
cbar.set_label('Depth [m]')
plt.show()

# Create figure and get axes
fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')

# Plot 3D point cloud with a colormap along depth axis
ax.scatter( entry[:,0], entry[:,1], entry[:,4], c= entry[:,4], cmap="gnuplot")

ax.set_aspect("equal")
ax.view_init(azim=-90, elev=-90)
# Set labels
ax.set_xlabel('X Label')
ax.set_ylabel('Y Label')
ax.set_zlabel('Z Label')

# Display result
plt.show()

# Create another figure to plot the histogram
fig = plt.figure()
ax = fig.add_subplot(111)
N, bins, patches = ax.hist(entry[:,4], bins=45, edgecolor='white', linewidth=1)

z_bool = (entry[:,4] < (min(entry[:,4])+0.25*(max(entry[:,4])-min(entry[:,4]))))
entry = entry[z_bool,:]
mean_Z = np.mean(entry[:,4])
plt.plot([mean_Z, mean_Z],[0, max(N)+50],linewidth=3,color='darkred')
plt.text(mean_Z, max(N)+75, 'Mean of 25% closest pixels', horizontalalignment='center', color="darkred")
plt.xticks(list(plt.xticks()[0]) + [mean_Z])
ax.set_xlabel('Depth [m]')
ax.set_ylabel('Number of pixels')
make_square_axes(ax)
for i in range(0,int(45*0.25)):
    patches[i].set_facecolor('royalblue')
for i in range(int(45*0.25),len(patches)):   
    patches[i].set_facecolor('grey')    
plt.plot
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

# np.mean(entry[:,4])
# 2.3763650048335863

    
    
