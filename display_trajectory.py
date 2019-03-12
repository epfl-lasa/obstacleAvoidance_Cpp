
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as anim
import matplotlib.patches as mpatches
import glob
import imageio


def disp1(): # Display the whole trajectory of a bunch of points on a single picture
    plt.figure()
    
    for name in glob.glob("./traj*.txt"):
    
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
    circle3 = plt.Circle((4, 0.5), 1, color='b')
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
    ax.add_artist(circle3)
    #plt.axis('scaled')
    ax.set_xlim(-2, 8)
    ax.set_ylim(-2, 8)
    plt.show()


"""import glob, os
os.chdir("/mydir")
for file in glob.glob("*.txt"):
    print(file)"""
    
    
### GIF GENERATION

def disp2(): # Create an animated gif displaying the movement of points/obstacles over time
    data_obstacles = np.loadtxt(open("obstacles_trajectory.txt", "rb"), delimiter=",")
    data_obstacles[:,0] = np.round(data_obstacles[:,0],2)
    
    flag = True 
    names = glob.glob("./100trajecto*.txt")
    for name in names:
        entry = np.loadtxt(open(name, "rb"), delimiter=",")
        if flag: 
            data_point = entry
            flag = False
        else: 
            data_point = np.vstack((data_point, entry))
    
    #data_point = np.loadtxt(open(names[1], "rb"), delimiter=",")
    data_point[:,0] = np.round(data_point[:,0],2)
    flag = True
    names = glob.glob("./10trajecto*.txt")
    for name in names:
        entry = np.loadtxt(open(name, "rb"), delimiter=",")
        if flag: 
            data_point_bis = entry
            flag = False
        else: 
            data_point_bis = np.vstack((data_point_bis, entry))
    data_point_bis[:,0] = np.round(data_point_bis[:,0],2)
            
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
            # Plot the position of the robot for that timestamp
            robot = data_point_bis[data_point_bis[:,0] == time_entry];
            for i_row in range(0,robot.shape[0]):
                plt.plot( robot[i_row,1], robot[i_row,2], 'bx', LineWidth=2)
                
            ax.set_xlim(-2, 8)
            ax.set_ylim(-2, 8)
            plt.title("Timestamp : " + str(time_entry) + " s")
            fig.savefig("img_gif/temp.png")
            plt.close()
            images.append(imageio.imread("img_gif/temp.png"))
    imageio.mimsave('moving_robot.gif', images, duration = 0.1)



     
def disp3(): # vector field to display velocity according to the position of the robot
    names = glob.glob("./quiver*.txt")
    names = glob.glob("./test_qui*.txt")
    names = glob.glob("./MultiplicationData/quiver_mul*.txt")
    names = glob.glob("./quiver_data_bor*.txt")
    for name in names:
        entry = np.loadtxt(open(name, "rb"), delimiter=",")
        X = entry[:,0]
        Y = entry[:,1]
        U = entry[:,2]
        V = entry[:,3]
        M = np.hypot(U, V)
    
    fig, ax = plt.subplots()
    q = ax.quiver(X, Y, U, V, M)
    #ellipse = mpatches.Rectangle([2,2], 1,1)
    #ax.add_artist(ellipse)
    ellipse = mpatches.Ellipse([17,6], 0.2, 0.2)
    ax.add_artist(ellipse)
    ax.set_xlim(0, 20)
    ax.set_ylim(0, 20)
    plt.show()
    
from mpl_toolkits.mplot3d import Axes3D
from matplotlib import cm
from matplotlib.ticker import LinearLocator, FormatStrFormatter
def disp4(): # vector field to display velocity according to the position of the robot
    names = glob.glob("./test*.txt")
    for name in names:
        entry = np.loadtxt(open(name, "rb"), delimiter=",")
        X = np.unique(entry[:,0])
        Y = np.unique(entry[:,1])
        X, Y = np.meshgrid(X, Y)
        Z = np.reshape(entry[:,2], (Y.shape[0],X.shape[0]), order='F')
        Z = np.round(Z, 2)
        print(type(Z))
    fig = plt.figure()
    ax = fig.gca(projection='3d')
    
    # Plot the surface.
    surf = ax.plot_surface(X, Y, Z, cmap=cm.coolwarm,linewidth=0,antialiased=False)
                
    # Customize the z axis
#    ax.set_zlim(-1.01, 1.01)
    ax.zaxis.set_major_locator(LinearLocator(10))
    ax.zaxis.set_major_formatter(FormatStrFormatter('%.02f'))
                    
    # Add a color bar which maps values to colors.
    fig.colorbar(surf, shrink=0.5, aspect=5)
    
    plt.show()
       
def disp5():
    from mpl_toolkits.mplot3d import Axes3D
    import matplotlib.pyplot as plt
    from matplotlib import cm
    from matplotlib.ticker import LinearLocator, FormatStrFormatter
    import numpy as np
    
    
    fig = plt.figure()
    ax = fig.gca(projection='3d')
    
    # Make data.
    X = np.arange(-5, 5, 0.25)
    Y = np.arange(-5, 5, 0.25)
    X, Y = np.meshgrid(X, Y)
    R = np.sqrt(X**2 + Y**2)
    Z = np.sin(R)
    print(Z)
    
    # Plot the surface.
    surf = ax.plot_surface(X, Y, Z, cmap=cm.coolwarm,
                        linewidth=0, antialiased=False)
    
    # Customize the z axis.
    ax.set_zlim(-1.01, 1.01)
    ax.zaxis.set_major_locator(LinearLocator(10))
    ax.zaxis.set_major_formatter(FormatStrFormatter('%.02f'))
    
    # Add a color bar which maps values to colors.
    fig.colorbar(surf, shrink=0.5, aspect=5)
    
    plt.show()

def disp6(): # vector field to display velocity according to the position of the robot
    names = glob.glob("./quiver_data_bor*.txt")
    names_pt = glob.glob("./quiver_points_bor*.txt")
    for name in names:
        entry = np.loadtxt(open(name, "rb"), delimiter=",")
        X = entry[:,0]
        Y = entry[:,1]
        U = entry[:,2]
        V = entry[:,3]
        M = np.hypot(U, V)
        
    fig, ax = plt.subplots()
    q = ax.quiver(X, Y, U, V, M)
    
    ellipse = mpatches.Ellipse([17,6], 0.2, 0.2)
    ax.add_artist(ellipse)
    
    ellipse = mpatches.Rectangle([3.5,4.5], 1, 1)
    ax.add_artist(ellipse)
    
    for name in names_pt:
        entry = np.loadtxt(open(name, "rb"), delimiter=",")
        X = (entry[:,0]).transpose()
        Y = (entry[:,1]).transpose()
        plt.plot(X,Y,'ko')
        
    ax.set_xlim(0, 20)
    ax.set_ylim(0, 14)
    plt.show()

def disp7(): # Create an animated gif displaying the movement of points/obstacles over time FOR BORDER
    names_pt = glob.glob("./quiver_points_bor*.txt")
    names_obs = glob.glob("./Trajectories/obs_data.txt")
    
    flag = True 
    names = glob.glob("./Trajectories/trajecto*.txt")
    for name in names:
        entry = np.loadtxt(open(name, "rb"), delimiter=",")
        if flag: 
            data_point = entry
            flag = False
        else: 
            data_point = np.vstack((data_point, entry))
    data_point[:,0] = np.round(data_point[:,0],2)
            
    timestamps = np.unique(data_point[:,0])
    
    images = []
    for time_entry in timestamps:
        print( time_entry, "/", max(timestamps))
        if True:# (np.remainder(time_entry,0.05)==0):
            fig = plt.figure()
            ax = fig.gca()
            
            # Plot the points on the border
            for name in names_pt:
                entry = np.loadtxt(open(name, "rb"), delimiter=",")
                X = (entry[:,0]).transpose()
                Y = (entry[:,1]).transpose()
                plt.plot(X,Y,'ko')
            
            for name_obs in names_obs:
                obstacles = np.loadtxt(open(name_obs, "rb"), delimiter=",")
                for i_row in range(0,obstacles.shape[0]):
                    ellipse = mpatches.Rectangle([obstacles[i_row,0]-0.5,obstacles[i_row,1]-0.5], 1, 1)
                    ax.add_artist(ellipse)
            
            # Plot the position of the robot for that timestamp
            robot = data_point[data_point[:,0] == time_entry];
            for i_row in range(0,robot.shape[0]):
                plt.plot( robot[i_row,1], robot[i_row,2], 'ro', LineWidth=2)
                
            ellipse = mpatches.Ellipse([17,6], 0.5, 0.5, color='g')
            ax.add_artist(ellipse)
    
            ax.set_xlim(0, 20)
            ax.set_ylim(0, 14)
            plt.title("Timestamp : " + str(time_entry) + " s")
            fig.savefig("img_gif/temp.png")
            plt.close()
            images.append(imageio.imread("img_gif/temp.png"))
    imageio.mimsave('moving_robot.gif', images, duration = 0.1)

                  
def disp8(): # SHOW BORDER OF GROWING OBSTACLE
    #names_pt = glob.glob("./Obstacles/obs*.txt")
    #names_pt.sort()
    names_border = glob.glob("./Obstacles/several_bor*.txt")
    names_border.sort()
    names_obs = glob.glob("./Obstacles/growing_obs_80.txt")
    n_max = 80
    images = []
    
    for nobs in range(1,n_max):
        print(str(nobs) + "/" + str(n_max))
        fig = plt.figure()
        ax = fig.gca()
            
        """names_pts = names_pt[nobs-2]
        points_border = np.loadtxt(open(names_pts, "rb"), delimiter=",")
        #for i_row in range(0,points_border.shape[0]):
        plt.plot( points_border[:,0].transpose(), points_border[:,1].transpose(), 'ko', LineWidth=2)"""
    
        
        for name_obs in names_obs:
            obstacles = np.loadtxt(open(name_obs, "rb"), delimiter=",")
            for i_row in range(0,nobs):
                rectangle = mpatches.Rectangle([obstacles[i_row,0]-0.5,obstacles[i_row,1]-0.5], 1, 1)
                ax.add_artist(rectangle)
                
        name_bor = names_border[nobs-1]
    #for name_bor in names_border:
        borders = np.loadtxt(open(name_bor, "rb"), delimiter=",")
        for i_row in range(0,borders.shape[0]):
            if borders[i_row,2] == 1:
                if borders[i_row,3] == 0:
                    plt.plot([borders[i_row,0]-0.5, borders[i_row,0]+0.5], [borders[i_row,1], borders[i_row,1]], color="k", LineWidth=2)
                else:
                    plt.plot([borders[i_row,0], borders[i_row,0]], [borders[i_row,1]-0.5, borders[i_row,1]+0.5], color="k", LineWidth=2)
            elif borders[i_row,2] == 2:
                if borders[i_row,4] == 0:
                    arc = mpatches.Arc([borders[i_row,0]-0.5, borders[i_row,1]-0.5],1, 1, 0, 0, 90, LineWidth=2)
                elif borders[i_row,4] == 1:
                    arc = mpatches.Arc([borders[i_row,0]+0.5, borders[i_row,1]-0.5],1, 1, 0, 90, 180, LineWidth=2)
                elif borders[i_row,4] == 2:
                    arc = mpatches.Arc([borders[i_row,0]+0.5, borders[i_row,1]+0.5],1, 1, 0, 180, 270, LineWidth=2)
                elif borders[i_row,4] == 3:
                    arc = mpatches.Arc([borders[i_row,0]-0.5, borders[i_row,1]+0.5],1, 1, 0, 270, 360, LineWidth=2)
                ax.add_artist(arc)
            elif borders[i_row,2] == 3:
                if borders[i_row,4] == 0:
                    arc = mpatches.Arc([borders[i_row,0]-0.5, borders[i_row,1]-0.5],1, 1, 0, 0, 90, LineWidth=2)
                elif borders[i_row,4] == 1:
                    arc = mpatches.Arc([borders[i_row,0]+0.5, borders[i_row,1]-0.5],1, 1, 0, 90, 180, LineWidth=2)
                elif borders[i_row,4] == 2:
                    arc = mpatches.Arc([borders[i_row,0]+0.5, borders[i_row,1]+0.5],1, 1, 0, 180, 270, LineWidth=2)
                elif borders[i_row,4] == 3:
                    arc = mpatches.Arc([borders[i_row,0]-0.5, borders[i_row,1]+0.5],1, 1, 0, 270, 360, LineWidth=2)
                ax.add_artist(arc)

        ax.set_xlim(0, 20)
        ax.set_ylim(0, 20)
        plt.title("Size : " + str(nobs))
        fig.savefig("img_gif/temp.png")
        fig.savefig("img_gif/step_"+str(nobs)+".png")
        plt.close()
        images.append(imageio.imread("img_gif/temp.png"))
    imageio.mimsave('growing_obstacle.gif', images, duration = 0.5)

disp8()

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
