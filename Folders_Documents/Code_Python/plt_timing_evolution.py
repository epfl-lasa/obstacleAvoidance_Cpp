import numpy as np
import matplotlib.pyplot as plt
import glob

names = glob.glob("/home/leziart/catkin_ws/src/process_occupancy_grid/src/Logging/timing_gmapping_*.txt")
names.sort()
data_gmapping = np.loadtxt(open(names[-1], "rb"), delimiter=",")

names = glob.glob("/home/leziart/catkin_ws/src/process_occupancy_grid/src/Logging/timing_functions_*.txt")
names.sort()
data_functions = np.loadtxt(open(names[-1], "rb"), delimiter=",")

plt.figure()
plt.plot(data_gmapping[1:,0],data_gmapping[1:,1])
for i in range(9):
    data = data_functions[data_functions[:,0]==i]
    plt.plot(data[1:,1], data[1:,2])
  
plt.legend(["Gmapping node", "Parameters", "Retrieving map by calling gmapping service", "Processing robot pos", "Processing attrac pos", "Drawing people on map", "Processing occupancy grid", "Detecting obstacles + boundaries", "Computing velocity command", "Sending velocity command"])
plt.xlabel("Time [s]")
plt.ylabel("Processing time [s]")
plt.title("Processing time of each step of the obstacle avoidance algorithm")
plt.show()

## Average time of each step

data_functions_mean = np.zeros(9)
for i in range(9):
    data = data_functions[data_functions[:,0]==i]
    data_functions_mean[i] = np.mean(data[:,2])
    
plt.figure()
plt.plot(["Parameters", "Retrieving map by\n calling gmapping service", "Processing\n robot pos", "Processing\n attrac pos", "Drawing\n people on map", "Processing\n occupancy grid", "Detecting\n obstacles + boundaries", "Computing\n velocity command", "Sending\n velocity command"], data_functions_mean, 'o')
plt.grid(True)
plt.xlabel("Name of the step")
plt.ylabel("Processing time [s]")
plt.title("Processing time of each step of the obstacle avoidance algorithm")
plt.show()

## Average time of the loop

timestamps = np.unique(data_functions[data_functions[:,0]==0,1])
data_functions_sum = np.zeros(len(timestamps))
for i in range(len(timestamps)):
    data = data_functions[(data_functions[:,1]==timestamps[i]) & (data_functions[:,0]!=5) & (data_functions[:,0]!=6)]
    data_functions_sum[i] = np.sum(data[:,2])
    
data_functions_sum[0] = data_functions_sum[1] # Fix problem for first value

average = np.mean(data_functions_sum)    
percentile95 = np.percentile(data_functions_sum,99)
plt.figure()
plt.plot(timestamps, data_functions_sum, linewidth=2)
plt.plot(timestamps, average*np.ones(len(timestamps)), linewidth=2, color='red')
plt.plot(timestamps, percentile95*np.ones(len(timestamps)), linewidth=2, color='forestgreen')
plt.legend(["Processing time of the whole loop", "Average of the processing time", "99-th percentile of the processing time"])
plt.grid(True)
plt.xlabel("Time of the simulation [s]")
plt.ylabel("Processing time [s]")
plt.title("Evolution of the processing time of a loop over the simulation")
plt.show()