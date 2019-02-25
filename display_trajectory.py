
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as anim
import matplotlib.patches as mpatches
import glob
import imageio

plt.figure()

for name in glob.glob("./t*.txt"):

    file = open(name, "r") 
    traj = file.readlines() 
    file.close()
    
    x = []
    y = []
    
    i = 0 
    for line in traj:
        point = line.split(',')
        x.append(float(point[1]))
        y.append(float(point[2]))
        i = i + 1
        
    plt.plot(x,y, LineStyle='--', LineWidth=2)
    plt.plot( x[0], y[0], 'ro', LineWidth=2)
    plt.plot( 4,2, 'go', LineWidth=2)
    


circle1 = plt.Circle((0, 0), 1, color='b')
circle2 = plt.Circle((2, 2), 1, color='b')
#fig, ax = plt.subplots() # note we must use plt.subplots, not plt.subplot
# (or if you have an existing figure)
fig = plt.gcf()
ax = fig.gca()

# Plotting Ellipses
patches = []
ellipse = mpatches.Ellipse([0,0], 1.2, 2, 45)
patches.append(ellipse)
ax.add_artist(ellipse)

#ax.add_artist(circle1)
ax.add_artist(circle2)
#plt.axis('scaled')
ax.set_xlim(-2, 5)
ax.set_ylim(-2, 5)
plt.show()


"""import glob, os
os.chdir("/mydir")
for file in glob.glob("*.txt"):
    print(file)"""
    
    
### GIF GENERATION

data_obstacles = np.loadtxt(open("obstacles_trajectory.txt", "rb"), delimiter=",")
data_obstacles[:,0] = np.round(data_obstacles[:,0],2)

flag = True 
names = glob.glob("./t*.txt")
for name in names:
    entry = np.loadtxt(open(name, "rb"), delimiter=",")
    if flag: 
        data_point = entry
        flag = False
    else: 
        data_point = np.vstack((data_point, entry))
    
#data_point = np.loadtxt(open(names[1], "rb"), delimiter=",")
data_point[:,0] = np.round(data_point[:,0],2)

timestamps = np.unique(np.hstack((data_point[:,0], data_obstacles[:,0])))

images = []
for time_entry in timestamps:
    if True:# (np.remainder(time_entry,0.05)==0):
        fig = plt.figure()
        ax = fig.gca()
        
        # Plot all obstacles for that timestamp
        obstacles = data_obstacles[data_obstacles[:,0] == time_entry];
        for i_row in range(0,obstacles.shape[0]):
            ellipse = mpatches.Ellipse([obstacles[i_row,1],obstacles[i_row,2]], 2*obstacles[i_row,4], 2*obstacles[i_row,5], (180/3.1415)*obstacles[i_row,3])
            ax.add_artist(ellipse)
            
        # Plot the position of the robot for that timestamp
        robot = data_point[data_point[:,0] == time_entry];
        for i_row in range(0,robot.shape[0]):
            plt.plot( robot[i_row,1], robot[i_row,2], 'ro', LineWidth=2)
        
            
        ax.set_xlim(-2, 8)
        ax.set_ylim(-2, 8)
        fig.savefig("img_gif/temp.png")
        plt.close()
        images.append(imageio.imread("img_gif/temp.png"))
imageio.mimsave('moving_robot.gif', images, duration = 0.1)


"""
# Save figures to a given directory
for time_entry in timestamps:
    fig = plt.figure()
    ax = fig.gca()
    
    for name in glob.glob("./t*.txt"):
    
    # Plotting Ellipses
    ellipse = mpatches.Ellipse([i*0.1,0.5], 1.2, 2, 45)
    ax.add_artist(ellipse)
    ax.set_xlim(-2, 10)
    ax.set_ylim(-3, 3)
    if (i < 10):
        fig.savefig("img_gif/img_00" + str(i) + ".png")
    elif (i < 100):
        fig.savefig("img_gif/img_0" + str(i) + ".png")
    else:
        fig.savefig("img_gif/img_" + str(i) + ".png")
    plt.close()

# Create gif by reading pictures in the directory

images = []
filenames = glob.glob("./img_gif/*.png")
filenames.sort()
for filename in filenames:
    images.append(imageio.imread(filename))
imageio.mimsave('movie.gif', images, duration = 0.1)"""
