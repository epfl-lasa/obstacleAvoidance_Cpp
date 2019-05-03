## P-A Leziart, 16 April 2019
# Goal: Graphic representation of the projection of an initial space into a circle space
# as well as the representation of trajectories in both space at the same time

from matplotlib import pyplot as plt
import numpy as np
import glob

# Detect text files for morphing
name_start = glob.glob("/home/leziart/Documents/Project_01_04/Data_Initial_Space_bis*.txt")
name_end   = glob.glob("/home/leziart/Documents/Project_01_04/Data_Circle_Space_bis*.txt")

# Detect text files for morphing for trajectories
name_traj_start_1 = glob.glob("/home/leziart/Documents/Project_01_04/Trajectory_Initial_Space.txt")
name_traj_end_1   = glob.glob("/home/leziart/Documents/Project_01_04/Trajectory_Circle_Space.txt")

name_traj_start_2 = glob.glob("/home/leziart/Documents/Project_01_04/Trajectory_Initial_Space_2.txt")
name_traj_end_2   = glob.glob("/home/leziart/Documents/Project_01_04/Trajectory_Circle_Space_2.txt")

# Extract data from text files for morphing
points_start = np.loadtxt(open(name_start[0], "rb"), delimiter=",")
points_end   = np.loadtxt(open(name_end[0], "rb"), delimiter=",")

# Extract data from text files for trajectories
traj_start_1 = np.loadtxt(open(name_traj_start_1[0], "rb"), delimiter=",")
traj_end_1   = np.loadtxt(open(name_traj_end_1[0], "rb"), delimiter=",")
traj_start_1 = traj_start_1.transpose()
traj_end_1 = traj_end_1.transpose()

traj_start_2 = np.loadtxt(open(name_traj_start_2[0], "rb"), delimiter=",")
traj_end_2   = np.loadtxt(open(name_traj_end_2[0], "rb"), delimiter=",")
traj_start_2 = traj_start_2.transpose()
traj_end_2 = traj_end_2.transpose()

# Remove points inside obstacle
points_start = points_start[(points_end[:,0]<500)]
points_end = points_end[(points_end[:,0]<500)]

# Create colormap based on angle in circle space
colors = np.arctan2( (points_end[:,1]), (points_end[:,0]) )
colors += min(colors)

### DISPLAY INITIAL SPACE AND TRAJECTORIES IN INITIAL SPACE

# Get figure and its axes
fig = plt.figure()
ax = fig.gca()

# Plot the initial space with a colormap based on their projection in the circle space
points_frame = points_start.transpose()
c = ax.scatter(points_frame[0,:], points_frame[1,:], c=colors, cmap='gnuplot') 

# Plot the separation at half perimeter from the attractor
x = points_start[:,0]
y = points_start[:,1]
test_x = x[(np.abs(points_end[:,1]) < 0.01)  & (points_end[:,0]<0)]
test_y = y[(np.abs(points_end[:,1]) < 0.01)  & (points_end[:,0]<0)]
plt.plot(test_x, test_y, "o", color="violet")

# Plot trajectory number 1
plt.plot(traj_start_1[0,:], traj_start_1[1,:], "x", color="forestgreen")
# Plot trajectory number 2
#plt.plot(traj_start_2[0,:], traj_start_2[1,:], "x", color="cornflowerblue")
# Plot attractor
#plt.plot(8,1.6, "Xk", markersize=10)
plt.plot(14.3, 6.6, "Xk", markersize=10)

# Plot legend
#plt.legend(["Critical points", "Robot 1", "Robot 2", "Attractor"])
plt.legend(["Critical points", "Robot 1", "Attractor"])

# Labels, title and limits
plt.xlabel("Position along X")
plt.ylabel("Position along Y")
plt.title("Trajectories in the initial space")
plt.xlim([1, 17.02])
plt.ylim([0, 13.02])

# Display the result
plt.show()

# Save result to png
#fig.savefig("initial_space.png", dpi=800)

### DISPLAY CIRCLE SPACE AND TRAJECTORIES IN CIRCLE SPACE

# Get figure and its axes
fig = plt.figure()
ax = fig.gca()

# Plot the initial space with a colormap based on their projection in the circle space
points_frame = points_end.transpose()
c = ax.scatter(points_frame[0,:], points_frame[1,:], c=colors, cmap='gnuplot') 

# Plot the separation at half perimeter from the attractor
x = points_end[:,0]
y = points_end[:,1]
test_x = x[(np.abs(points_end[:,1]) < 0.01)   & (points_end[:,0]<0)]
test_y = y[(np.abs(points_end[:,1]) < 0.01)   & (points_end[:,0]<0)]
plt.plot(test_x, test_y, "o", color="violet")

# Plot trajectory number 1
plt.plot(traj_end_1[0,:], traj_end_1[1,:], "x", color="forestgreen")
# Plot trajectory number 2
#plt.plot(traj_end_2[0,:], traj_end_2[1,:], "x", color="cornflowerblue")
# Plot attractor
#plt.plot(2.4,0, "Xk", markersize=10)
#plt.plot(2.67,0, "Xk", markersize=10)
plt.plot(1.55,0, "Xk", markersize=10)

# Plot legend
#plt.legend(["Critical points", "Robot 1", "Robot 2", "Attractor"])
plt.legend(["Critical points", "Robot 1", "Attractor"])

# Labels, title and limits
plt.xlabel("Position along X")
plt.ylabel("Position along Y")
plt.title("Trajectories in the circle space")
plt.xlim([-4.5, 4.5])
plt.ylim([-4.5, 4.5])

# Display the result
plt.show()

# Save result to png
#fig.savefig("circle_space.png", dpi=800)