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


#### PARAMETERS

radius_ridgeback = 0.6;
size_cell = 0.3;
radius_in_grid = int(math.ceil(radius_ridgeback/size_cell))
####
names = glob.glob("/home/leziart/catkin_ws/src/process_occupancy_grid/src/Logging/data_obstacles_1558616704.txt")
names = glob.glob("/home/leziart/catkin_ws/src/process_occupancy_grid/src/Logging/data_obstacles_1558627328.txt")
names = glob.glob("/home/leziart/catkin_ws/src/process_occupancy_grid/src/Logging/data_obstacles_1558627456.txt")
names.sort()
data = np.loadtxt(open(names[0], "rb"), delimiter=",")


timestamps = np.unique(data[:,0])

pos = np.random.random((len(timestamps),4))
person1 = np.random.random((len(timestamps),4))
person2 = np.random.random((len(timestamps),4))


class UpdatablePatchCollection(mcollections.PatchCollection):
    def __init__(self, patches, *args, **kwargs):
        self.patches = patches
        mcollections.PatchCollection.__init__(self, patches, *args, **kwargs)

    def get_paths(self):
        self.set_paths(self.patches)
        return self._paths
        

fig_circle =  plt.figure()
plt.subplot(211)
plt.plot(pos[0:3,1], pos[0:3,2], lw=2)
ax = plt.axis([-4,4,-4,4])
axes = plt.gca() 
axes.set_aspect("equal")
plt.subplot(212)
plt.plot(pos[0:3,1], pos[0:3,2], lw=2)
ax = plt.axis([-4,4,-4,4])
axes = plt.gca() 
axes.set_aspect("equal")


fig, ax = plt.subplots()

l, = plt.plot(pos[0:1,1], pos[0:1,2], lw=2)
ellipse1 = mpatches.Ellipse([person1[1,1],person1[1,2]], 1, 1, facecolor="red", zorder=2)
ellipse2 = mpatches.Ellipse([person2[1,1],person2[1,2]], 1, 1, facecolor="red", zorder=2)
lst = []
lst.append(ellipse1)
lst.append(ellipse2)

collection_feat1 = UpdatablePatchCollection([])
#ax.add_collection(collection_feat1)

collection_feat2 = UpdatablePatchCollection([])
ax.add_collection(collection_feat2)

ax = plt.axis([330,375,300,355])

axes = plt.gca() 
axes.set_aspect("equal")
axes.set_title("Time [s]: " + str(round(pos[1,0],3)))

axamp = plt.axes([0.25, .03, 0.50, 0.02])

# Slider
initial_cursor = 0
samp = Slider(axamp, 'Step', 0, len(timestamps)-1, valinit=initial_cursor)

def update(val):
    axes.artists = []
    
    # cursor is the current value of the slider
    cursor = int(samp.val)
    data_x = pos[0:cursor,1]
    data_y = pos[0:cursor,2]
    # update curve
    l.set_xdata(data_x)
    l.set_ydata(data_y)
    # update ellipses
    ellipse1.set_center((person1[cursor,1],person1[cursor,2]))
    ellipse2.set_center((person2[cursor,1],person2[cursor,2]))
    
    # Update features data
    data_feat1 = data[(data[:,0]==timestamps[cursor]) & (data[:,2]==1)]
    data_feat2 = data[(data[:,0]==timestamps[cursor]) & (data[:,2]==2)]
    data_feat3 = data[(data[:,0]==timestamps[cursor]) & (data[:,2]==3)]
    data_feat4 = data[(data[:,0]==timestamps[cursor]) & (data[:,2]==4)]
    data_feat5 = data[(data[:,0]==timestamps[cursor]) & (data[:,2]==5)]
    data_feat6 = data[(data[:,0]==timestamps[cursor]) & (data[:,2]==6)]
    data_feat7 = data[(data[:,0]==timestamps[cursor]) & (data[:,2]==7)]
    data_feat8 = data[(data[:,0]==timestamps[cursor]) & (data[:,2]==8)]
    
    # Update occupied cells (feature 1)
    for i in range(data_feat1.shape[0]):
        occupied_cell = mpatches.Rectangle([data_feat1[i,3]-0.5,data_feat1[i,4]-0.5], 1, 1)
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
            
    # Update position of the projection of the robot in initial space (feature 3)
    for i in range(data_feat3.shape[0]):
        circle = mpatches.Ellipse([data_feat3[i,3],data_feat3[i,4]], 1, 1, facecolor="red")        
        axes.add_artist(circle)
    
    # Update position of the projection of the attractor in initial space (feature 4)
    for i in range(data_feat4.shape[0]):
        circle = mpatches.Ellipse([data_feat4[i,3],data_feat4[i,4]], 1, 1, facecolor="forestgreen")        
        axes.add_artist(circle)
    
    # Update position of the robot in initial space (feature 5)
    if (data_feat5.shape[0] > 0):
        circle = mpatches.Ellipse([data_feat5[0,3],data_feat5[0,4]], 2*radius_in_grid, 2*radius_in_grid, facecolor="coral")        
        axes.add_artist(circle)
        
    # Update position of the attractor in initial space (feature 6)
    if (data_feat6.shape[0] > 0):
        circle = mpatches.Ellipse([data_feat6[0,3],data_feat6[0,4]], 1, 1, facecolor="forestgreen")       
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
            arrow = mpatches.FancyArrow(data_feat5[0,3],data_feat5[0,4], (K_mult/norm_vec)*data_feat7[i,3], (K_mult/norm_vec)*data_feat7[i,4], length_includes_head=True, head_width=0.4, Linewidth=2, facecolor="rebeccapurple")
            axes.add_artist(arrow)
    
    # Update trajectory of the robot in initial space (since its starting position)
    data_feat5_prev = data[(data[:,0]<=timestamps[cursor]) & (data[:,2]==5)]
    if (data_feat5.shape[0] > 0):
        line = mlines.Line2D(data_feat5_prev[:,3],data_feat5_prev[:,4], Linewidth=2, Linestyle="--",color='darkorange')
        axes.add_artist(line)
        
    # Update Gamma distance of the robot (feature 7)
    for i in range(data_feat8.shape[0]):
        x = (data_feat5[0,3]+data_feat3[i,3]) * 0.5
        y = (data_feat5[0,4]+data_feat3[i,4]) * 0.5
        text = mtext.Text(x,y,str(data_feat8[i,3]))
        axes.add_artist(text)

    # Update title
    fig.canvas.set_window_title("Time [s]: " + str(round(timestamps[cursor],3)))
    axes.set_title("Time [s]: " + str(round(timestamps[cursor],3)))
        
    # Redraw canvas while idle
    fig.canvas.draw_idle()

# call update function on slider value change
samp.on_changed(update)

plt.show()

