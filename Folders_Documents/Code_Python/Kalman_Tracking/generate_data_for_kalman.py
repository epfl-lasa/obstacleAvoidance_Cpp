## P-A Leziart, 16 April 2019
# Goal: Generate data to test the efficiency of a Kalman filter
# Write the data to a text file
# Read the text file from the C++ code, run the C++ Kalman filter and write the
# result into another text file
# Read that text file from Python and plot stuff with Matplotlib

import numpy as np
from matplotlib import pyplot as plt

## Generating data for C++ code

# Function to generate data and write it in a text file
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
    X_meas = X + np.random.normal(loc=0, scale=5, size=(n_steps))
    Y_meas = Y + np.random.normal(loc=0, scale=5, size=(n_steps))
    VX_meas = np.diff( np.hstack((X0,X_meas)))/h
    VY_meas = np.diff( np.hstack((Y0,Y_meas)))/h
    
    data = (np.vstack((X_meas, Y_meas, VX_meas, VY_meas))).transpose()
    
    #np.savetxt("data_from_python.txt", data, fmt='%.4f', delimiter=" ")
    
    #print(repr((np.vstack((X_meas, Y_meas, VX_meas, VY_meas))).transpose() ))
    
    # ################# #
    # SQUARE TRAJECTORY #
    # ################# #
    
    # Timestep
    h = 1/8
    
    
    # Movements
    X0 = 100
    Y0 = 100
    X1 = 200
    Y1 = 200
    Xi = []
    Yi = []
    Xi.append(np.linspace(X0,X1,8*2))
    Xi.append(np.linspace(X1,X1,8*2))
    Xi.append(np.linspace(X1,X0,8*2))
    Xi.append(np.linspace(X0,X0,8*2))
    
    Yi.append(np.linspace(Y0,Y0,8*2))
    Yi.append(np.linspace(Y0,Y1,8*2))
    Yi.append(np.linspace(Y1,Y1,8*2))
    Yi.append(np.linspace(Y1,Y0,8*2))
    
    X = []
    Y = []
    for i in range(len(Xi)): X = np.hstack((X,Xi[i]))
    for i in range(len(Yi)): Y = np.hstack((Y,Yi[i]))
    Vx = np.hstack(([0],np.diff(X)/h))
    Vy = np.hstack(([0],np.diff(Y)/h))
    
    # Add noise to measurements
    np.random.seed(42)
    std_dev = 5
    X_meas = X + np.random.normal(loc=0, scale=std_dev, size=len(X))
    Y_meas = Y + np.random.normal(loc=0, scale=std_dev, size=len(Y))
    VX_meas = np.diff( np.hstack((X0,X_meas)))/h
    VY_meas = np.diff( np.hstack((Y0,Y_meas)))/h
    
    data = (np.vstack((X_meas, Y_meas, VX_meas, VY_meas))).transpose()
    
    np.savetxt("data_from_python.txt", data, fmt='%.4f', delimiter=" ", newline='\n')
    
    return X, Y, Vx, Vy, X_meas, Y_meas, VX_meas, VY_meas
    
    

## Reading data from C++ code

import glob

def reading_data():
    global X, Y, Vx, Vy, X_meas, Y_meas, VX_meas, VY_meas
    n_steps = len(X)
    
    T = np.linspace(0,8-0.125,8*2*4)
    
    names_cor = glob.glob("Data_Kalman_Cor*.txt")
    names_pred = glob.glob("Data_Kalman_Pred*.txt")
    for name in names_cor:
        entry = np.loadtxt(open(names_cor[0], "rb"), delimiter=",")
        #print(entry)
        
        entry_pred = np.loadtxt(open(names_pred[0], "rb"), delimiter=",")
        
        plt.subplot(221)
        plt.plot(T,X, color='k')
        plt.plot(T,X_meas, color='orangered')
        plt.plot(T,(entry_pred[:,0]).transpose(), color='violet')
        plt.plot(T,(entry[:,0]).transpose(), color='forestgreen')
        plt.title("Position along X over time")
        plt.xlabel("Time [s]")
        plt.ylabel("Position along X [pixels]")
        plt.legend(["Ground Truth", "Measurements", "Predicted (Kalman)", "Corrected (Kalman)"])
        
        
        plt.subplot(222)
        plt.plot(T,Y, color='k')
        plt.plot(T,Y_meas, color='orangered')
        plt.plot(T,(entry_pred[:,1]).transpose(), color='violet')
        plt.plot(T,(entry[:,1]).transpose(), color='forestgreen')
        plt.title("Position along Y over time")
        plt.xlabel("Time [s]")
        plt.ylabel("Position along Y [pixels]")
        plt.legend(["Ground Truth", "Measurements", "Predicted (Kalman)", "Corrected (Kalman)"])
        
        
        plt.subplot(223)
        plt.plot(T,Vx, color='k')
        plt.plot(T,VX_meas, color='orangered')
        plt.plot(T,(entry_pred[:,2]).transpose(), color='violet')
        plt.plot(T,(entry[:,2]).transpose(), color='forestgreen')
        plt.title("Speed along X over time")
        plt.xlabel("Time [s]")
        plt.ylabel("Speed along X [pixels/s]")
        plt.legend(["Ground Truth", "Measurements", "Predicted (Kalman)", "Corrected (Kalman)"])
        
        
        plt.subplot(224)
        plt.plot(T,Vy, color='k')
        plt.plot(T,VY_meas, color='orangered')
        plt.plot(T,(entry_pred[:,3]).transpose(), color='violet')
        plt.plot(T,(entry[:,3]).transpose(), color='forestgreen')
        plt.title("Speed along Y over time")
        plt.xlabel("Time [s]")
        plt.ylabel("Speed along Y [pixels/s]")
        plt.legend(["Ground Truth", "Measurements", "Predicted (Kalman)", "Corrected (Kalman)"])
        plt.show()
        
        print(np.max(np.abs(entry[:,0]-X)))
        print(np.max(np.abs(entry[:,1]-Y)))

## Call functions

#[X, Y, Vx, Vy, X_meas, Y_meas, VX_meas, VY_meas] = generating_data()

# RUN C++ CODE

#reading_data()

