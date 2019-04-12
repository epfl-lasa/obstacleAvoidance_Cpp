import numpy as np
from matplotlib import pyplot as plt

def generating_data():
    h = 0.1
    X0 = 100
    Y0 = 150
    vX = np.hstack((10 * np.ones(30), 0 * np.ones(30), -10 * np.ones(30)))
    vY = np.hstack((10 * np.ones(30), 0 * np.ones(30), -10 * np.ones(30)))
    n_steps = 90
    X = X0 + np.cumsum(vX * h)
    Y = Y0 + np.cumsum(vY * h)
    #X = X0 + vX * np.arange(1,n_steps+1,1) * h
    #Y = Y0 + vY * np.arange(1,n_steps+1,1) * h
    
    # Add noise to measurements
    np.random.seed(42)
    X_meas = X + np.random.normal(loc=0, scale=2, size=(n_steps))
    Y_meas = Y + np.random.normal(loc=0, scale=2, size=(n_steps))
    VX_meas = np.diff( np.hstack((X0,X_meas)))/h
    VY_meas = np.diff( np.hstack((Y0,Y_meas)))/h
    
    data = (np.vstack((X_meas, Y_meas, VX_meas, VY_meas))).transpose()
    
    np.savetxt("data_from_python.txt", data, fmt='%.4f', delimiter=" ")
    
    #print(repr((np.vstack((X_meas, Y_meas, VX_meas, VY_meas))).transpose() ))

## Reading data from C++ code

import glob

def reading_data():
    names_cor = glob.glob("Data_Kalman_Cor*.txt")
    names_pred = glob.glob("Data_Kalman_Pred*.txt")
    for name in names_cor:
        entry = np.loadtxt(open(names_cor[0], "rb"), delimiter=",")
        print(entry)
        
        entry_pred = np.loadtxt(open(names_pred[0], "rb"), delimiter=",")
        
        plt.subplot(221)
        plt.plot(X, color='k')
        plt.plot(X_meas, color='orangered')
        plt.plot((entry_pred[:,0]).transpose(), color='violet')
        plt.plot((entry[:,0]).transpose(), color='forestgreen')
        plt.title("Position along X over time")
        plt.xlabel("Step")
        plt.ylabel("Position along X [pixels]")
        plt.legend(["Ground Truth", "Measurements", "Predicted (Kalman)", "Corrected (Kalman)"])
        plt.show()
        
        plt.subplot(222)
        plt.plot(Y, color='k')
        plt.plot(Y_meas, color='orangered')
        plt.plot((entry_pred[:,1]).transpose(), color='violet')
        plt.plot((entry[:,1]).transpose(), color='forestgreen')
        plt.title("Position along Y over time")
        plt.xlabel("Step")
        plt.ylabel("Position along Y [pixels]")
        plt.legend(["Ground Truth", "Measurements", "Predicted (Kalman)", "Corrected (Kalman)"])
        plt.show()
        
        plt.subplot(223)
        plt.plot(vX * np.ones(n_steps), color='k')
        plt.plot(VX_meas, color='orangered')
        plt.plot((entry_pred[:,2]).transpose(), color='violet')
        plt.plot((entry[:,2]).transpose(), color='forestgreen')
        plt.title("Speed along X over time")
        plt.xlabel("Step")
        plt.ylabel("Speed along X [pixels/s]")
        plt.legend(["Ground Truth", "Measurements", "Predicted (Kalman)", "Corrected (Kalman)"])
        plt.show()
        
        plt.subplot(224)
        plt.plot(vY * np.ones(n_steps), color='k')
        plt.plot(VY_meas, color='orangered')
        plt.plot((entry_pred[:,3]).transpose(), color='violet')
        plt.plot((entry[:,3]).transpose(), color='forestgreen')
        plt.title("Speed along Y over time")
        plt.xlabel("Step")
        plt.ylabel("Speed along Y [pixels/s]")
        plt.legend(["Ground Truth", "Measurements", "Predicted (Kalman)", "Corrected (Kalman)"])
        plt.show()
    
## Visualization of depth point cloud

from mpl_toolkits.mplot3d import Axes3D
from scipy.signal import find_peaks

#def depth_point_cloud():
name = glob.glob("/home/leziart/Downloads/data_pro*.txt")
name = name[0]
entry = np.loadtxt(open(name, "rb"), delimiter=",")

entry = entry[~np.isnan(entry).any(axis=1)]

indexes = np.arange(0, entry.shape[0], 1)
indexes = (indexes%10)==0

# Take only the upper portion of the body
y_limit = int(640 * 0.2) 
y_bool = (entry[:,1] < y_limit)

entry = entry[indexes & y_bool,:]

hist, bin_edges  = np.histogram(entry[:,2], bins=30)

fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')

ax.scatter( entry[:,0], entry[:,1], entry[:,2], c= entry[:,2], cmap="gnuplot")

ax.set_xlabel('X Label')
ax.set_ylabel('Y Label')
ax.set_zlabel('Z Label')

plt.show()

fig = plt.figure()
ax = fig.add_subplot(111)
ax.hist(entry[:,2], bins=30)
plt.show()

plt.figure()
x = hist
peaks, properties = find_peaks(x, prominence=1)
plt.plot(x)
plt.plot(peaks, x[peaks], "x")
plt.vlines(x=peaks, ymin=x[peaks] - properties["prominences"], ymax = x[peaks], color = "C1")
#plt.hlines(y=properties["width_heights"], xmin=properties["left_ips"], xmax=properties["right_ips"], color = "C1")
plt.show()

#depth_point_cloud()
    
    
