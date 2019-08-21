import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as anim
import matplotlib.patches as mpatches
import matplotlib.lines as mlines
import matplotlib.text as mtext
import glob

import matplotlib.animation as animation
from matplotlib.widgets import Slider
import matplotlib.collections as mcollections
import math


## Parameters

radius_ridgeback = 0.6
size_cell = 0.2
radius_in_grid = int(math.ceil(radius_ridgeback/size_cell))
limit_in_meters = math.sqrt(50-1)
limit_in_cells  = int(math.ceil(limit_in_meters))


## Loading files
# names = glob.glob("/home/leziart/catkin_ws/src/process_occupancy_grid/src/Logging/data_obstacles_1558616704.txt")
# names = glob.glob("/home/leziart/catkin_ws/src/process_occupancy_grid/src/Logging/data_obstacles_1558627328.txt")
# names = glob.glob("/home/leziart/catkin_ws/src/process_occupancy_grid/src/Logging/data_obstacles_1558627456.txt")
# names = glob.glob("/home/leziart/catkin_ws/src/process_occupancy_grid/src/Logging/data_obstacles_1558689280.txt")
# names = glob.glob("/home/leziart/catkin_ws/src/process_occupancy_grid/src/Logging/data_obstacles_1558693376.txt")
# names = glob.glob("/home/leziart/catkin_ws/src/process_occupancy_grid/src/Logging/data_obstacles_1558701056.txt")
# names = glob.glob("/home/leziart/catkin_ws/src/process_occupancy_grid/src/Logging/data_obstacles_1558706176.txt")
num = str(1560418816)
num = str(1562242560)
system_name = "qolo"
names = glob.glob("/home/"+system_name+"/catkin_ws/src/process_occupancy_grid/src/Logging/data_obstacles_*.txt")
names = np.sort(names)
data = np.loadtxt(open(names[-1], "rb"), delimiter=",")

names = glob.glob("/home/"+system_name+"/catkin_ws/src/process_occupancy_grid/src/Logging/data_blobs_*.txt")
names = np.sort(names)
refresh_node = np.loadtxt(open(names[-1], "rb"), delimiter=",")

names = glob.glob("/home/"+system_name+"/catkin_ws/src/process_occupancy_grid/src/Logging/data_robot_*.txt")
names = np.sort(names)
robot = np.loadtxt(open(names[-1], "rb"), delimiter=",")

timestamps = np.unique(data[:,0])


## Updatable PatchCollection class

class UpdatablePatchCollection(mcollections.PatchCollection):
    def __init__(self, patches, *args, **kwargs):
        self.patches = patches
        mcollections.PatchCollection.__init__(self, patches, *args, **kwargs)

    def get_paths(self):
        self.set_paths(self.patches)
        return self._paths
        
## First figure to display velocity command and velocity of the robot
vel_cmd_data = robot[(robot[:,1]==6)]
vel_robot_data = robot[(robot[:,1]==3)]

fig_vel, (ax1_vel, ax2_vel) = plt.subplots(2)
ax1_vel.plot(vel_cmd_data[:,0],vel_cmd_data[:,2], color="orangered")
ax1_vel.plot(vel_robot_data[:,0],vel_robot_data[:,2], color="forestgreen")
ax1_vel.set(xlabel='Time [s]', ylabel='Velocity along X [m/s]')
ax1_vel.legend(["Velocity command", "Velocity of the robot"])
ax2_vel.plot(vel_cmd_data[:,0],vel_cmd_data[:,3], color="orangered")
ax2_vel.plot(vel_robot_data[:,0],vel_robot_data[:,3], color="forestgreen")
ax2_vel.set(xlabel='Time [s]', ylabel='Velocity along Y [m/s]')   
ax2_vel.legend(["Velocity command", "Velocity of the robot"])
plt.show()

## Second figure to display the position of the robot in circle space

circle_axes = []
#data = data[data[:,1]<=2] # Remove false obstacles (I will need to fix that)

if int(np.max(data[:,1])) > 0:
    sub_fig, axs = plt.subplots(int(np.max(data[:,1])))
    if int(np.max(data[:,1])) == 1: axs = [axs]
    for i in range(int(np.max(data[:,1]))):
        axs[i].set_aspect("equal")
        axs[i].set_xlim([-4,4])
        axs[i].set_ylim([-4,4])
        circle_axes.append(axs[i])


## Third figure to display the position of the robot in the occupancy grid

fig, ax = plt.subplots()
ax = plt.axis([75,125,250,280])
axes = plt.gca() 
axes.set_aspect("equal")

axamp = plt.axes([0.25, .03, 0.50, 0.02])

colors = ['r','b','m','y','violet','orange','pink']

## Main loop to update what is displayed

init_limits = True

# Slider
initial_cursor = 0
samp = Slider(axamp, 'Step', 0, len(timestamps)-1, valinit=initial_cursor)

def update(val):
    global init_limits
    axes.artists = []
    
    # cursor is the current value of the slider
    cursor = int(samp.val)

    temp_sync = refresh_node[((refresh_node[:,2]==1) & (refresh_node[:,1]!=0)),0]
    bool_sync = (refresh_node[((refresh_node[:,2]==1) & (refresh_node[:,1]!=0)),0])<=timestamps[cursor]
    timestamp_sync = np.max(temp_sync[bool_sync])
 
    # Update
    data_feat1_all = refresh_node[(refresh_node[:,0]==timestamp_sync) & (refresh_node[:,2]==1) & (refresh_node[:,1]==0)]
    data_feat5_all = data[(data[:,0]==timestamps[cursor]) & (data[:,2]==5) & (data[:,1]==0)]
    data_feat6_all = data[(data[:,0]==timestamps[cursor]) & (data[:,2]==6) & (data[:,1]==0)]
    #data_feat5_all = refresh_node[(refresh_node[:,0]==timestamp_sync) & (refresh_node[:,2]==5) & (refresh_node[:,1]==0)]
    #data_feat6_all = refresh_node[(refresh_node[:,0]==timestamp_sync) & (refresh_node[:,2]==6) & (refresh_node[:,1]==0)]
    
    # Update features from data_obstacles_XXX.txt
    
    data_feat1 = refresh_node[(refresh_node[:,0]==timestamp_sync) & (refresh_node[:,2]==1) & (refresh_node[:,1]!=0)]
    data_feat2 = refresh_node[(refresh_node[:,0]==timestamp_sync) & (refresh_node[:,2]==2) & (refresh_node[:,1]!=0)]
    data_feat3 = data[(data[:,0]==timestamps[cursor]) & (data[:,2]==3) & (data[:,1]!=0)]
    data_feat4 = data[(data[:,0]==timestamps[cursor]) & (data[:,2]==4) & (data[:,1]!=0)]
    data_feat5 = data[(data[:,0]==timestamps[cursor]) & (data[:,2]==5) & (data[:,1]!=0)]
    data_feat6 = data[(data[:,0]==timestamps[cursor]) & (data[:,2]==6) & (data[:,1]!=0)]
    data_feat7 = data[(data[:,0]==timestamps[cursor]) & (data[:,2]==7) & (data[:,1]!=0)]
    data_feat8 = data[(data[:,0]==timestamps[cursor]) & (data[:,2]==8) & (data[:,1]!=0)]
    data_feat20 = data[(data[:,0]==timestamps[cursor]) & (data[:,2]==20) & (data[:,1]!=0)] # Obstacle is in an obstacle or not
    
    #plt.axis([np.min(data_feat1_all[:,3])-3,np.max(data_feat1_all[:,3])+3,np.min(data_feat1_all[:,4])-3,np.max(data_feat1_all[:,4])+3])
    
    # Update features from data_robot_XXX.txt
    timestamp_for_robot = robot[np.argmin(np.abs(robot[:,0]-timestamps[cursor])),0]
    robot_feat6 = robot[(robot[:,0]==timestamp_for_robot) & (robot[:,1]==6)]
    robot_feat11 = robot[(robot[:,0]==timestamp_for_robot) & (robot[:,1]==11)]
    robot_feat12 = robot[(robot[:,0]<=timestamp_for_robot) & (robot[:,0]>=(timestamp_for_robot-0.25)) & (robot[:,1]==12)]
    
    # Update occupied cells (in range and out of range)
    for i in range(data_feat1_all.shape[0]):
        occupied_cell = mpatches.Rectangle([data_feat1_all[i,3]-0.5,data_feat1_all[i,4]-0.5], 1, 1, facecolor="c", zorder=5)
        axes.add_artist(occupied_cell)
    if (data_feat1_all.shape[0])>0 and (init_limits):
        axes.set_xlim([np.min(data_feat1_all[:,3])-8, np.max(data_feat1_all[:,3])+8])
        axes.set_ylim([np.min(data_feat1_all[:,4])-8, np.max(data_feat1_all[:,4])+8])
        init_limits = False
        
    # Update occupied cells (feature 1)
    for i in range(data_feat1.shape[0]):
        occupied_cell = mpatches.Rectangle([data_feat1[i,3]-0.5,data_feat1[i,4]-0.5], 1, 1,zorder=1, facecolor=colors[np.mod(int(data_feat1[i,1]),7)])
        axes.add_artist(occupied_cell)

    # Update boundary cells (feature 2)
    for i_row in range(0,data_feat2.shape[0]):
        if data_feat2[i_row,5] == 1:
            if data_feat2[i_row,6] == 0:
                line = mlines.Line2D([data_feat2[i_row,3]-0.5,data_feat2[i_row,3]+0.5],[data_feat2[i_row,4],data_feat2[i_row,4]], Linewidth=2,color='k')
            else:
                line = mlines.Line2D([data_feat2[i_row,3],data_feat2[i_row,3]],[data_feat2[i_row,4]-0.5,data_feat2[i_row,4]+0.5], Linewidth=2,color='k')
            axes.add_artist(line)
        elif data_feat2[i_row,5] == 2:
            if data_feat2[i_row,7] == 0:
                arc = mpatches.Arc([data_feat2[i_row,3]-0.5, data_feat2[i_row,4]-0.5],1, 1, 0, 0, 90, Linewidth=2)
            elif data_feat2[i_row,7] == 1:
                arc = mpatches.Arc([data_feat2[i_row,3]+0.5, data_feat2[i_row,4]-0.5],1, 1, 0, 90, 180, Linewidth=2)
            elif data_feat2[i_row,7] == 2:
                arc = mpatches.Arc([data_feat2[i_row,3]+0.5, data_feat2[i_row,4]+0.5],1, 1, 0, 180, 270, Linewidth=2)
            elif data_feat2[i_row,7] == 3:
                arc = mpatches.Arc([data_feat2[i_row,3]-0.5, data_feat2[i_row,4]+0.5],1, 1, 0, 270, 360, Linewidth=2)
            axes.add_artist(arc)
        elif data_feat2[i_row,5] == 3:
            if data_feat2[i_row,7] == 0:
                arc = mpatches.Arc([data_feat2[i_row,3]-0.5, data_feat2[i_row,4]-0.5],1, 1, 0, 0, 90, Linewidth=2)
            elif data_feat2[i_row,7] == 1:
                arc = mpatches.Arc([data_feat2[i_row,3]+0.5, data_feat2[i_row,4]-0.5],1, 1, 0, 90, 180, Linewidth=2)
            elif data_feat2[i_row,7] == 2:
                arc = mpatches.Arc([data_feat2[i_row,3]+0.5, data_feat2[i_row,4]+0.5],1, 1, 0, 180, 270, Linewidth=2)
            elif data_feat2[i_row,7] == 3:
                arc = mpatches.Arc([data_feat2[i_row,3]-0.5, data_feat2[i_row,4]+0.5],1, 1, 0, 270, 360, Linewidth=2)
            axes.add_artist(arc)
      
    # Update feature 11
    for i in range(robot_feat11.shape[0]):
        print("Feat 11: ", robot_feat11[i,2], " and ", robot_feat11[i,3])
        circle = mpatches.Ellipse([robot_feat11[i,2],robot_feat11[i,3]], 1, 1, facecolor="violet", edgecolor="k", zorder=8)        
        axes.add_artist(circle)
        
        # Update trajectory
        robot_feat11_prev = robot[(robot[:,0]<=timestamp_for_robot) & (robot[:,1]==11)]
        line = mlines.Line2D(robot_feat11_prev[:,2],robot_feat11_prev[:,3], Linewidth=2, Linestyle="--",color='darkviolet')
        axes.add_artist(line)
        
    # Update feature 12
    for i in range(robot_feat12.shape[0]):
        #print("Feat 12: ", robot_feat12[i,2], " and ", robot_feat12[i,3])
        circle = mpatches.Ellipse([robot_feat12[i,2],robot_feat12[i,3]], 1, 1, facecolor="violet", edgecolor="k", zorder=9)        
        axes.add_artist(circle)
        
    # Update trajectory
    robot_feat12_prev = robot[(robot[:,0]<=timestamp_for_robot) & (robot[:,1]==12)]
    for i in range(robot_feat12_prev.shape[0]):
        color_dot = np.array([1., 0., 0.])
        k = i / robot_feat12_prev.shape[0]
        circle = mpatches.Ellipse([robot_feat12_prev[i,2],robot_feat12_prev[i,3]], 0.8, 0.8, facecolor=k*color_dot, edgecolor="k", zorder=7)      
        axes.add_artist(circle)
      
            
    # Update position of the projection of the robot in initial space (feature 3)
    for i in range(data_feat3.shape[0]):
        circle = mpatches.Ellipse([data_feat3[i,3],data_feat3[i,4]], 1, 1, facecolor="red", edgecolor="k", zorder=3)        
        axes.add_artist(circle)
            
    
    # Update position of the projection of the attractor in initial space (feature 4)
    for i in range(data_feat4.shape[0]):
        circle = mpatches.Ellipse([data_feat4[i,3],data_feat4[i,4]], 1, 1, facecolor="forestgreen", edgecolor="k", zorder=2)        
        axes.add_artist(circle)
    
    # Update position of the robot in initial space (feature 5)
    if (data_feat5.shape[0] > 0):
        circle = mpatches.Ellipse([data_feat5[0,3],data_feat5[0,4]], 1, 1, facecolor="red", edgecolor="k", zorder=4)        
        axes.add_artist(circle)
        circle = mpatches.Ellipse([data_feat5[0,3],data_feat5[0,4]], 2*radius_in_grid, 2*radius_in_grid, fill=False, linestyle="--", edgecolor="k", zorder=4)        
        axes.add_artist(circle)
    else:
        circle = mpatches.Ellipse([data_feat5_all[0,3],data_feat5_all[0,4]], 1, 1, facecolor="red", edgecolor="k", zorder=4)        
        axes.add_artist(circle)
        circle = mpatches.Ellipse([data_feat5_all[0,3],data_feat5_all[0,4]], 2*radius_in_grid, 2*radius_in_grid, fill=False, linestyle="--", edgecolor="k", zorder=4)        
        axes.add_artist(circle)
        
    # Update position of the attractor in initial space (feature 6)
    if (data_feat6.shape[0] > 0):
        circle = mpatches.Ellipse([data_feat6[0,3],data_feat6[0,4]], 1, 1, facecolor="forestgreen", edgecolor="k", zorder=2)       
        axes.add_artist(circle)
    else:
        circle = mpatches.Ellipse([data_feat6_all[0,3],data_feat6_all[0,4]], 1, 1, facecolor="forestgreen", edgecolor="k", zorder=2)       
        axes.add_artist(circle)
     
    # Draw projection line of robot
    if (data_feat5.shape[0] > 0):
        for i in range(data_feat3.shape[0]):
            line = mlines.Line2D([data_feat5[0,3],data_feat3[i,3]],[data_feat5[0,4],data_feat3[i,4]], Linewidth=2, Linestyle="--", color='k')
            axes.add_artist(line)
        
    # Draw projection line of attractor    
    if (data_feat6.shape[0] > 0):
        for i in range(data_feat4.shape[0]):
            line = mlines.Line2D([data_feat6[0,3],data_feat4[i,3]],[data_feat6[0,4],data_feat4[i,4]], Linewidth=2, Linestyle="--",color='k')
            axes.add_artist(line)
    
    # Update velocity command of the robot (feature 7)
    if (data_feat5.shape[0] > 0):
        for i in range(data_feat7.shape[0]):
            K_mult = 3
            norm_vec = np.sqrt(np.sum(np.power(data_feat7[i,3:5],2)))
            arrow = mpatches.FancyArrow(data_feat5[0,3],data_feat5[0,4], K_mult*data_feat7[i,3], K_mult*data_feat7[i,4], length_includes_head=True, width=0.1, Linewidth=2, facecolor="rebeccapurple", zorder=5)
            axes.add_artist(arrow)
    
    # Update trajectory of the robot in initial space (since its starting position)
    data_feat5_prev = data[(data[:,0]<=timestamps[cursor]) & (data[:,2]==5) & (data[:,2]!=0)]
    if (data_feat5.shape[0] > 0):
        line = mlines.Line2D(data_feat5_prev[:,3],data_feat5_prev[:,4], Linewidth=2, Linestyle="--",color='darkorange')
        axes.add_artist(line)
    else:
        data_feat5_prev = data[(data[:,0]<=timestamps[cursor]) & (data[:,2]==5) & (data[:,2]==0)]
        line = mlines.Line2D(data_feat5_prev[:,3],data_feat5_prev[:,4], Linewidth=2, Linestyle="--",color='darkorange')
        axes.add_artist(line)
        
        
    # Update Gamma distance of the robot (feature 7)
    for i in range(data_feat8.shape[0]):
        x = (data_feat5[0,3]+data_feat3[i,3]) * 0.5 + 1
        y = (data_feat5[0,4]+data_feat3[i,4]) * 0.5 + 0.5
        text = mtext.Text(x,y,str(data_feat8[i,3]))
        axes.add_artist(text)

    # Update limit range to consider obstacles
    if (data_feat5.shape[0] > 0):
        circle = mpatches.Ellipse([data_feat5[0,3],data_feat5[0,4]], 2*limit_in_cells, 2*limit_in_cells, fill=False, edgecolor="k", zorder=2)       
        axes.add_artist(circle)
        rec = mpatches.Rectangle([data_feat5[0,3]-limit_in_cells,data_feat5[0,4]-limit_in_cells], 2*limit_in_cells, 2*limit_in_cells, fill=False, edgecolor="k", zorder=2)       
        axes.add_artist(rec)
                

    # Update velocity command sent to the robot (feature 6 of data_robot)
    if (robot_feat6.shape[0] > 0) and (data_feat5_all.shape[0] > 0) and not ((robot_feat6[0,2]==0)and(robot_feat6[0,3]==0)):
        K_mult = 3
        arrow = mpatches.FancyArrow(data_feat5_all[0,3],data_feat5_all[0,4], K_mult*robot_feat6[0,2], K_mult*robot_feat6[0,3], length_includes_head=True, width=0.1, Linewidth=2, facecolor="orange", edgecolor="k", zorder=6)
        axes.add_artist(arrow)

    # Update title to display Time and if the robot is in an obstacle or not
    if (data_feat20.shape[0] > 0):
        if np.any(data_feat20[:,3]==1.0):
            fig.canvas.set_window_title("Time [s]: " + str(round(timestamps[cursor],3)) + " - IN OBSTACLE")
            axes.set_title("Time [s]: " + str(round(timestamps[cursor],3)) + " - IN OBSTACLE")
        else:
            fig.canvas.set_window_title("Time [s]: " + str(round(timestamps[cursor],3)) + " - NOT IN OBSTACLE")
            axes.set_title("Time [s]: " + str(round(timestamps[cursor],3)) + " - NOT IN OBSTACLE")
    else:
        # Update title
        fig.canvas.set_window_title("Time [s]: " + str(round(timestamps[cursor],3)))
        axes.set_title("Time [s]: " + str(round(timestamps[cursor],3)))
    
    # Redraw canvas while idle
    fig.canvas.draw_idle()
    
    # Update circle figures
    if ((data_feat3[:,1]).shape[0] > 0):
        N_subplots = int(np.max(data_feat3[:,1]))
        for k_ax in range(len(circle_axes)):
            sub_fig.delaxes(circle_axes[-1])
            circle_axes.pop()
        sub_fig.suptitle(str(N_subplots) + " obstacle(s) considered")
        sub_fig.canvas.draw()
        for k_ax in range(N_subplots):
            ax = sub_fig.add_subplot(int("1"+str(N_subplots)+str(len(circle_axes)+1)))
            circle_axes.append(ax)
        sub_fig.canvas.draw()
    else:
        N_subplots = 0
        for k_ax in range(len(circle_axes)):
            sub_fig.delaxes(circle_axes[-1])
            circle_axes.pop()
        sub_fig.suptitle("No obstacle considered (inside obstacle or out of range)")
        sub_fig.canvas.draw()
        
    for i in range(len(circle_axes)): 
        axes_circle = circle_axes[i] 
        axes_circle.artists = []
        if np.any(data_feat3[:,1]==(i+1)):
            
            # Plot the unit circle
            unit_circle = mpatches.Ellipse([0,0], 2, 2, facecolor="royalblue", edgecolor="k", zorder=1) 
            axes_circle.add_artist(unit_circle)
            
            # Update position of the projection of the robot in circle space for this obstacle (feature 3)
            circle = mpatches.Ellipse([data_feat3[i,5],data_feat3[i,6]], 0.2, 0.2, facecolor="red", edgecolor="k", zorder=3)         
            axes_circle.add_artist(circle)
            
            # Update position of the projection of the attractor in circle space for this obstacle (feature 4)
            circle = mpatches.Ellipse([data_feat4[i,5],data_feat4[i,6]], 0.2, 0.2, facecolor="forestgreen", edgecolor="k", zorder=2)         
            axes_circle.add_artist(circle)
            
            # Limits of the canvas
            maxi = np.max([np.abs(data_feat5[i,5]),np.abs(data_feat5[i,6])])
            axes_circle.set_xlim([-maxi*1.3,maxi*1.3])
            axes_circle.set_ylim([-maxi*1.3,maxi*1.3])
            
            # Update position of the robot in circle space for this obstacle (feature 5)
            circle = mpatches.Ellipse([data_feat5[i,5],data_feat5[i,6]], 0.1*maxi, 0.1*maxi, facecolor="red", edgecolor="k", zorder=4)          
            axes_circle.add_artist(circle)
                
            # Update position of the attractor in circle space for this obstacle (feature 6)
            circle = mpatches.Ellipse([data_feat6[i,5],data_feat6[i,6]], 0.2, 0.2, facecolor="forestgreen", edgecolor="k", zorder=2)         
            axes_circle.add_artist(circle)
        
            # Draw projection line of robot
            line = mlines.Line2D([data_feat5[i,5],data_feat3[i,5]],[data_feat5[i,6],data_feat3[i,6]], Linewidth=2, Linestyle="--",    color='k')
            axes_circle.add_artist(line)
                
            # Draw projection line of attractor    
            line = mlines.Line2D([data_feat6[i,5],data_feat4[i,5]],[data_feat6[i,6],data_feat4[i,6]], Linewidth=2, Linestyle="--", color='k')
            axes_circle.add_artist(line)
            
            # Update velocity command of the robot (feature 7)
            K_mult = maxi*0.3 # width=0.07
            norm_vec = np.sqrt(np.sum(np.power(data_feat7[i,5:7],2)))
            arrow = mpatches.FancyArrow(data_feat5[i,5],data_feat5[i,6], K_mult*data_feat7[i,5], K_mult*data_feat7[i,6], length_includes_head=True, width=maxi*0.02, Linewidth=2, facecolor="rebeccapurple", zorder=5)
            axes_circle.add_artist(arrow)
            
            # Update trajectory of the robot in initial space (since its starting position)
            data_feat5_prev = data[(data[:,0]<=timestamps[cursor]) & (data[:,2]==5) & (data[:,1]==(i+1))]
            line = mlines.Line2D(data_feat5_prev[:,5],data_feat5_prev[:,6], Linewidth=2, Linestyle="--",color='darkorange')
            axes_circle.add_artist(line)
        
            # Update title
            axes_circle.set_title("Time [s]: " + str(round(timestamps[cursor],3)) + " | Obstacle " + str(i+1) + " | Distance: " + str(data_feat8[i,4]))
    
    sub_fig.canvas.draw_idle()
    
    
    # Draw a red dot on the plot with velocity along X
    """lim_X = ax1_vel.get_xlim()
    lim_Y = ax1_vel.get_ylim()
    circle = mpatches.Ellipse([timestamps[cursor],robot_feat6[0,2]],(lim_X[1]-lim_X[0])/120, (lim_Y[1]-lim_Y[0])/40, facecolor="red", edgecolor="k", zorder=4)          
    ax1_vel.artists = []
    ax1_vel.add_artist(circle)
    
    # Draw a red dot on the plot with velocity along Y
    lim_X = ax2_vel.get_xlim()
    lim_Y = ax2_vel.get_ylim()
    circle = mpatches.Ellipse([timestamps[cursor],robot_feat6[0,3]], (lim_X[1]-lim_X[0])/120, (lim_Y[1]-lim_Y[0])/40, facecolor="red", edgecolor="k", zorder=4)          
    ax2_vel.artists = []
    ax2_vel.add_artist(circle)"""
    
    # Draw changes
    fig_vel.canvas.draw_idle()
    
    # Go back to main figure
    plt.figure(fig.number)

# call update function on slider value change
samp.on_changed(update)

plt.show()

