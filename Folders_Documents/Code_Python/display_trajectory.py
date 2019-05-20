## File to display different things with Matplotlib by reading txt file containing
# comma-separated data that has been written by the C++ code

import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as anim
import matplotlib.patches as mpatches
import glob
import imageio


def disp1():
    """
    Plot the trajectories of a bunch of robots
    Each robot has its trajectory stored in its own file
    Each line is associated with a point of the trajectories 
    Each line contains "Timestamp, x_pos, y_pos"
    """
    
    plt.figure()
    
    # For each robot, reading its associated text file
    for name in glob.glob("./traj*.txt"):
        entry = np.loadtxt(open(name, "rb"), delimiter=",")
        x = (entry[:,1]).transpose()
        y = (entry[:,2]).transpose()
            
        plt.plot(x,y, LineStyle='--', LineWidth=2)
        plt.plot( x[0], y[0], 'ro', LineWidth=2) # Plot the starting position of the robot
    
    plt.plot( 4,2, 'go', LineWidth=2) # Plot the position of the attractor
        
    # Get figure and axes handles
    fig = plt.gcf()
    ax = fig.gca()
    
    # Define obstacles, here they are circles or ellipses
    circle1 = plt.Circle((0, 0), 1, color='b')
    circle2 = plt.Circle((2, 2), 1, color='b')
    circle3 = plt.Circle((4, 0.5), 1, color='b')
    ellipse = mpatches.Ellipse([0,0], 1.2, 2, 45)
    
    # Plot obstacles
    #ax.add_artist(circle1)
    #ax.add_artist(circle2)
    ax.add_artist(circle3)
    ax.add_artist(ellipse)
    
    # Set axis limits and display result
    ax.set_xlim(-2, 8)
    ax.set_ylim(-2, 8)
    plt.show()

    
### GIF GENERATION

def disp2(): 
    """
    Create an animated gif displaying the movement of robots and obstacles over time
    Each robot has its trajectory stored in its own file
    Obstacles have a single file where all their trajectories is stored
    Each line is associated with a position at a given moment in time
    Each line contains "Timestamp, x_pos, y_pos"
    """
    
    # Reading data of obstacles
    data_obstacles = np.loadtxt(open("obstacles_trajectory.txt", "rb"), delimiter=",")
    # Getting all the timestamps
    data_obstacles[:,0] = np.round(data_obstacles[:,0],2)
    
    # Reading data of robots and stacking all their trajectories in a single array (first experiment)
    flag = True 
    names = glob.glob("./100trajecto*.txt")
    for name in names:
        entry = np.loadtxt(open(name, "rb"), delimiter=",")
        if flag: # For the first robot the array is not created yet so we don't use vstack
            data_point = entry
            flag = False
        else:   # For the following the array is already created so we use vstack
            data_point = np.vstack((data_point, entry))
    # Rounding timestamps to 2 digits precision     
    data_point[:,0] = np.round(data_point[:,0],2)
    
    # Reading data of robots and stacking all their trajectories in a single array (second experiment)
    flag = True
    names = glob.glob("./10trajecto*.txt")
    for name in names:
        entry = np.loadtxt(open(name, "rb"), delimiter=",")
        if flag: # For the first robot the array is not created yet so we don't use vstack
            data_point_bis = entry
            flag = False
        else:    # For the following the array is already created so we use vstack
            data_point_bis = np.vstack((data_point_bis, entry))
    # Rounding timestamps to 2 digits precision 
    data_point_bis[:,0] = np.round(data_point_bis[:,0],2)
      
    # Stacking all timestamps and removing duplicates
    timestamps = np.unique(np.hstack((data_point[:,0], data_obstacles[:,0])))
    
    # Gif creation
    images = [] # Array that contains the frames
    for time_entry in timestamps:
        if True:# (np.remainder(time_entry,0.05)==0):
            fig = plt.figure()
            ax = fig.gca()
            
            # Plot all obstacles for that timestamp
            obstacles = data_obstacles[data_obstacles[:,0] == time_entry];
            for i_row in range(0,obstacles.shape[0]):
                ellipse = mpatches.Ellipse([obstacles[i_row,1],obstacles[i_row,2]], 2*obstacles[i_row,4], 2*obstacles[i_row,5], (180/3.1415)*obstacles[i_row,3])
                ax.add_artist(ellipse)
                
            # Plot the position of the robot for that timestamp (first experiment)
            robot = data_point[data_point[:,0] == time_entry];
            for i_row in range(0,robot.shape[0]):
                plt.plot( robot[i_row,1], robot[i_row,2], 'ro', LineWidth=2)
                
            # Plot the position of the robot for that timestamp (second experiment)
            robot = data_point_bis[data_point_bis[:,0] == time_entry];
            for i_row in range(0,robot.shape[0]):
                plt.plot( robot[i_row,1], robot[i_row,2], 'bx', LineWidth=2)
            
            # Set axes limits, title and save the frame in images list
            ax.set_xlim(-2, 8)
            ax.set_ylim(-2, 8)
            plt.title("Timestamp : " + str(time_entry) + " s")
            fig.savefig("img_gif/temp.png")
            plt.close()
            images.append(imageio.imread("img_gif/temp.png"))
    
    # Use all the frames to create a gif
    imageio.mimsave('moving_robot.gif', images, duration = 0.1)

     
def disp3(): 
    """
    Plot a vector field to display which velocity command is computed according to the position
    Each line is formatted as follows "x_pos, y_pos, x_vel, y_vel"
    Use the quiver function of Matplotlib
    """
    
    # Get the file that contains the data
    names = glob.glob("./quiver*.txt")
    names = glob.glob("./test_qui*.txt")
    names = glob.glob("./MultiplicationData/quiver_mul*.txt")
    names = glob.glob("./quiver_data_bor*.txt")
    
    # Read the Quiver text file that contains the data
    for name in names:
        entry = np.loadtxt(open(name, "rb"), delimiter=",")
        X = entry[:,0]
        Y = entry[:,1]
        U = entry[:,2]
        V = entry[:,3]
        M = np.hypot(U, V)
    
    # Create figure and get axes handle
    fig, ax = plt.subplots()
    
    # Plot the vector field
    q = ax.quiver(X, Y, U, V, M)
    
    # Plot the obstacle
    obstacle = mpatches.Rectangle([2,2], 1, 1)
    ax.add_artist(obstacle)
    
    # Plot the attractor
    ellipse = mpatches.Ellipse([17,6], 0.2, 0.2)
    ax.add_artist(ellipse)
    
    # Set axes and display figure
    ax.set_xlim(0, 20)
    ax.set_ylim(0, 20)
    plt.show()


from mpl_toolkits.mplot3d import Axes3D
from matplotlib import cm
from matplotlib.ticker import LinearLocator, FormatStrFormatter
def disp4():
    """
    Plot a surface to display which velocity command is computed according to the position
    Each line is formatted as follows "x_pos, y_pos, velocity_magnitude"
    Use the plot_surface function of Matplotlib
    """
    
    # Get data and put in the the correct format
    names = glob.glob("./test*.txt")
    for name in names:
        entry = np.loadtxt(open(name, "rb"), delimiter=",")
        X = np.unique(entry[:,0])
        Y = np.unique(entry[:,1])
        X, Y = np.meshgrid(X, Y)
        Z = np.reshape(entry[:,2], (Y.shape[0],X.shape[0]), order='F')
        Z = np.round(Z, 2)
        # print(type(Z))
        
    # Create figure and get axes handle
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
    
    # Display result
    plt.show()
       
def disp5():
    """
    Demo function as a reference about how to use the plot_surface function
    """
    
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
    # print(Z)
    
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

def disp6(): 
    """
    Plot a vector field to display which velocity command is computed according to the position
    Each line is formatted as follows "x_pos, y_pos, x_vel, y_vel"
    Use the quiver function of Matplotlib
    Seems to do the same things as disp3()
    """
    
    # Get data and put in the the correct format
    names = glob.glob("./quiver_data_bor*.txt")
    names_pt = glob.glob("./quiver_points_bor*.txt")
    names = glob.glob("./stream_data_bor*.txt")
    for name in names:
        entry = np.loadtxt(open(name, "rb"), delimiter=",")
        X = entry[:,0]
        Y = entry[:,1]
        U = entry[:,2]
        V = entry[:,3]
        M = np.hypot(U, V)
        
        
    fig, ax = plt.subplots()
    q = ax.quiver(X, Y, U, V, M)
    
    #ellipse = mpatches.Ellipse([17,6], 0.2, 0.2)
    #ax.add_artist(ellipse)
    
    #ellipse = mpatches.Rectangle([3.5,4.5], 1, 1)
    #ax.add_artist(ellipse)
    
    ellipse = mpatches.Ellipse([8,5], 0.1, 0.1, color='r')
    ax.add_artist(ellipse)
    """
    for name in names_pt:
        entry = np.loadtxt(open(name, "rb"), delimiter=",")
        X = (entry[:,0]).transpose()
        Y = (entry[:,1]).transpose()
        plt.plot(X,Y,'ko')"""
    
    ax.set_xlim(-9, 25)
    ax.set_ylim(-9, 25)    
    #ax.set_xlim(2.5, 15.5)
    #ax.set_ylim(1, 12)
    plt.show()

def disp6bis(): 
    """
    Plot a stream field to display the velocity flow in the workspace
    Each line is formatted as follows "x_pos, y_pos, x_vel, y_vel"
    Use the streamplot function of Matplotlib
    """
    names = glob.glob("./stream_data_bor*.txt")
    
    # Get data and put in the the correct format
    for name in names:
        entry = np.loadtxt(open(name, "rb"), delimiter=",")
        n_size = int(len(entry[:,0])**0.5)
        X = np.reshape(entry[:,0], (n_size,n_size)).transpose()
        Y = np.reshape(entry[:,1], (n_size,n_size)).transpose()
        U = np.reshape(entry[:,2], (n_size,n_size)).transpose()
        V = np.reshape(entry[:,3], (n_size,n_size)).transpose()
    
    # Create figure and get axes handle
    fig, ax = plt.subplots()
    
    # Plot the stream
    q = ax.streamplot(X, Y, U, V, density=6)
    
    # Plot the attractor
    ellipse = mpatches.Ellipse([8,1.6], 0.1, 0.1, color='r')
    ax.add_artist(ellipse)
     
    # Set axis limits and display result
    min_x = np.min(entry[:,0])
    max_x = np.max(entry[:,0])
    min_y = np.min(entry[:,1])
    max_y = np.max(entry[:,1])
    
    ax.set_xlim(min_x, max_x)
    ax.set_ylim(min_y, max_y)
    #ax.set_xlim(300, 390)
    #ax.set_ylim(280, 370)
    plt.show()
    
def disp7(): # Create an animated gif displaying the movement of points/obstacles over time FOR BORDER
    """
    Display the trajectories of robots avoiding an obstacle composed of cells
    Display the obstacle itself as well as the points on its border
    """
    
    names_pt = glob.glob("./quiver_points_bor*.txt")
    names_obs = glob.glob("./Trajectories/obs_data.txt")
    
    # Get the trajectory of the robots
    flag = True 
    names = glob.glob("./Trajectories/trajecto*.txt")
    for name in names:
        entry = np.loadtxt(open(name, "rb"), delimiter=",")
        if flag: 
            data_point = entry
            flag = False
        else: 
            data_point = np.vstack((data_point, entry))
    # Round the timestamps to 2 digits precision
    data_point[:,0] = np.round(data_point[:,0],2)
        
    # Remove timestamps duplicates
    timestamps = np.unique(data_point[:,0])
    
    images = []
    for time_entry in timestamps:
        print( time_entry, "/", max(timestamps))
        if True:# (np.remainder(time_entry,0.05)==0):
            fig = plt.figure()
            ax = fig.gca()
            
            # Plot the points on the border of the obstacle
            for name in names_pt:
                entry = np.loadtxt(open(name, "rb"), delimiter=",")
                X = (entry[:,0]).transpose()
                Y = (entry[:,1]).transpose()
                plt.plot(X,Y,'ko')
            
            # Plot the occupied cells that form the obstacle
            for name_obs in names_obs:
                obstacles = np.loadtxt(open(name_obs, "rb"), delimiter=",")
                for i_row in range(0,obstacles.shape[0]):
                    ellipse = mpatches.Rectangle([obstacles[i_row,0]-0.5,obstacles[i_row,1]-0.5], 1, 1)
                    ax.add_artist(ellipse)
            
            # Plot the position of the robots for that timestamp
            robot = data_point[data_point[:,0] == time_entry];
            for i_row in range(0,robot.shape[0]):
                plt.plot( robot[i_row,1], robot[i_row,2], 'ro', LineWidth=2)
            
            # Plot the attractor
            ellipse = mpatches.Ellipse([17,6], 0.5, 0.5, color='g')
            ax.add_artist(ellipse)
    
            # Set axes limits and save the frame in images list
            ax.set_xlim(0, 20)
            ax.set_ylim(0, 14)
            plt.title("Timestamp : " + str(time_entry) + " s")
            fig.savefig("img_gif/temp.png")
            plt.close()
            images.append(imageio.imread("img_gif/temp.png"))
            
    # Use all the frames to create a gif
    imageio.mimsave('moving_robot.gif', images, duration = 0.1)

                  
def disp8(): # SHOW BORDER OF GROWING OBSTACLE
    """
    Display how the border creation algorithm works
    An empty occupancy grid is progressively filled with occupied cells which are
    detected as obstacles
    """
    
    #names_pt = glob.glob("./Obstacles/obs*.txt")
    #names_pt.sort()
    names_border = glob.glob("./Obstacles/several_bor*.txt")
    names_border.sort()
    names_obs = glob.glob("./Obstacles/growing_obs_80.txt")
    n_max = 80
    images = []
    
    for nobs in range(1,n_max): # For each step (one step = one new occupied cell)
        print(str(nobs) + "/" + str(n_max))
        fig = plt.figure()
        ax = fig.gca()
            
        """names_pts = names_pt[nobs-2]
        points_border = np.loadtxt(open(names_pts, "rb"), delimiter=",")
        #for i_row in range(0,points_border.shape[0]):
        plt.plot( points_border[:,0].transpose(), points_border[:,1].transpose(), 'ko', LineWidth=2)"""
    
        # Plot occupied cells
        for name_obs in names_obs:
            obstacles = np.loadtxt(open(name_obs, "rb"), delimiter=",")
            for i_row in range(0,nobs):
                rectangle = mpatches.Rectangle([obstacles[i_row,0]-0.5,obstacles[i_row,1]-0.5], 1, 1)
                ax.add_artist(rectangle)
                
        name_bor = names_border[nobs-1]
    #for name_bor in names_border:
    
        # Load the border information which has been created by the C++ code
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

        # Set axis limits and save the frame in the images list
        ax.set_xlim(0, 20)
        ax.set_ylim(0, 20)
        plt.title("Size : " + str(nobs))
        fig.savefig("img_gif/temp.png")
        fig.savefig("img_gif/step_"+str(nobs)+".png")
        plt.close()
        images.append(imageio.imread("img_gif/temp.png"))
    
    # Use all the frames to create a gif
    imageio.mimsave('growing_obstacle.gif', images, duration = 0.5)

def disp9():
    """
    Display the occupied cells that form an obstacle
    """
    names_obs = glob.glob("./Obstacles/article_obs.txt")
    fig = plt.figure()
    ax = fig.gca()
    for name_obs in names_obs:
        obstacles = np.loadtxt(open(name_obs, "rb"), delimiter=",")
        for i_row in range(0,obstacles.shape[0]):
            rectangle = mpatches.Rectangle([obstacles[i_row,0]-0.5,obstacles[i_row,1]-0.5], 1, 1)
            ax.add_artist(rectangle)
    
    #ellipse = mpatches.Ellipse([8,1.5], 0.15, 0.15, color='r')
    #ax.add_artist(ellipse)
    
    """"names_border = glob.glob("./Obstacles/article_border.txt")
    borders = np.loadtxt(open(names_border[0], "rb"), delimiter=",")
    for i_row in range(0,borders.shape[0]):
        if borders[i_row,2] == 1:
            if borders[i_row,3] == 0:
                plt.plot([borders[i_row,0]-0.5, borders[i_row,0]+0.5], [borders[i_row,1], borders[i_row,1]], color="r", LineWidth=3)
            else:
                plt.plot([borders[i_row,0], borders[i_row,0]], [borders[i_row,1]-0.5, borders[i_row,1]+0.5], color="r", LineWidth=3)
        elif borders[i_row,2] == 2:
            if borders[i_row,4] == 0:
                arc = mpatches.Arc([borders[i_row,0]-0.5, borders[i_row,1]-0.5],1, 1, 0, 0, 90, LineWidth=3, color="r")
            elif borders[i_row,4] == 1:
                arc = mpatches.Arc([borders[i_row,0]+0.5, borders[i_row,1]-0.5],1, 1, 0, 90, 180, LineWidth=3, color="r")
            elif borders[i_row,4] == 2:
                arc = mpatches.Arc([borders[i_row,0]+0.5, borders[i_row,1]+0.5],1, 1, 0, 180, 270, LineWidth=3, color="r")
            elif borders[i_row,4] == 3:
                arc = mpatches.Arc([borders[i_row,0]-0.5, borders[i_row,1]+0.5],1, 1, 0, 270, 360, LineWidth=3, color="r")
            ax.add_artist(arc)
        elif borders[i_row,2] == 3:
            if borders[i_row,4] == 0:
                arc = mpatches.Arc([borders[i_row,0]-0.5, borders[i_row,1]-0.5],1, 1, 0, 0, 90, LineWidth=3, color="r")
            elif borders[i_row,4] == 1:
                arc = mpatches.Arc([borders[i_row,0]+0.5, borders[i_row,1]-0.5],1, 1, 0, 90, 180, LineWidth=3, color="r")
            elif borders[i_row,4] == 2:
                arc = mpatches.Arc([borders[i_row,0]+0.5, borders[i_row,1]+0.5],1, 1, 0, 180, 270, LineWidth=3, color="r")
            elif borders[i_row,4] == 3:
                arc = mpatches.Arc([borders[i_row,0]-0.5, borders[i_row,1]+0.5],1, 1, 0, 270, 360, LineWidth=3, color="r")
            ax.add_artist(arc)"""
                
    ax.set_xlim(2.5, 15.5)
    ax.set_ylim(1, 12)
    plt.show()

import time
def disp10(): 
    """
    Plot a streamplot for each position of the robot along a trajectory to show 
    the effect of udapting the occupancy map as only the obstacles in a given range
    around the robot are detected
    """
    
    names = glob.glob("/home/leziart/catkin_ws/StreamData/stream_data_bor*.txt")
    names.sort()
    
    positions = glob.glob("/home/leziart/catkin_ws/StreamData/stream_pos*.txt")
    positions.sort()
    
    names_bor = glob.glob("/home/leziart/catkin_ws/StreamData/stream_obs*.txt")
    names_bor.sort()
    
    images = []
    ntot = len(names)
    ncur = 0
    for name in names:
        
        posrobot = np.loadtxt(open(positions[ncur], "rb"), delimiter=",")
        
        entry = np.loadtxt(open(name, "rb"), delimiter=",")
        n_size = int(len(entry[:,0])**0.5)
        
        limit_dist_x = 7
        limit_dist_y = 7 
        #bool_entry = np.reshape(((abs(entry[:,0]- np.ones(entry.shape[0]) * posrobot[0]) <limit_dist_x) & (abs(entry[:,1]- np.ones(entry.shape[0]) * posrobot[1]) <limit_dist_y)), (n_size,n_size)).transpose()
        
        #entry = entry[((abs(entry[:,0]- np.ones(entry.shape[0]) * posrobot[0]) <limit_dist_x) & (abs(entry[:,1]- np.ones(entry.shape[0]) * posrobot[1]) <limit_dist_y)),:]
        #n_size = int(len(entry[:,0])**0.5)

        # Reshape data for the stream plot
        X = np.reshape(entry[:,0], (n_size,n_size)).transpose()
        Y = np.reshape(entry[:,1], (n_size,n_size)).transpose()
        U = np.reshape(entry[:,2], (n_size,n_size)).transpose()
        V = np.reshape(entry[:,3], (n_size,n_size)).transpose()
        
        # Get the obstacles in the limit range
        limit_dist = 7
        i_x = np.where(np.abs(X[0,:]-posrobot[0]) < limit_dist)[0]
        i_y = np.where(np.abs(Y[:,0]-posrobot[1]) < limit_dist)[0]

        X = X[i_y[0]:(i_y[-1]+1),i_x[0]:(i_x[-1]+1)]
        Y = Y[i_y[0]:(i_y[-1]+1),i_x[0]:(i_x[-1]+1)]
        U = U[i_y[0]:(i_y[-1]+1),i_x[0]:(i_x[-1]+1)]
        V = V[i_y[0]:(i_y[-1]+1),i_x[0]:(i_x[-1]+1)]

        # Create a figure and get axes handle
        fig, ax = plt.subplots()
        
        # Plot the borders of obstacles in range
        borders = np.loadtxt(open(names_bor[ncur], "rb"), delimiter=",")
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
        
        # Plot the velocity stream around the robot
        q = ax.streamplot(X, Y, U, V, density=0.5)
        
        # Plot the position of the attractor
        ellipse1 = mpatches.Ellipse([373,296.33], 2, 2, facecolor="orangered", zorder=2)
        ax.add_patch(ellipse1)
        
        # Plot the position of the robot during this frame
        ellipse2 = mpatches.Ellipse([posrobot[0],posrobot[1]], 2, 2, facecolor="forestgreen", zorder=2)
        ax.add_patch(ellipse2)
        
        # Sex axes limits, maximize the window to increase resolution and save the frame in images list
        ax.set_xlim(327, 377)
        ax.set_ylim(291, 341)
        manager = plt.get_current_fig_manager()
        manager.window.showMaximized()
        plt.show()
        time.sleep(0.1)
        plt.title("Step " + str(ncur))
        fig.savefig("img_gif/temp.png", dpi=200)
        plt.close()
        images.append(imageio.imread("img_gif/temp.png"))
        
        print(str(ncur) + "/" + str(ntot))
        ncur += 1
        
    # Use all the frames to create a gif    
    imageio.mimsave('moving_stream_9.gif', images, duration = 0.165)


def disp_debug():
    names1 = glob.glob("/home/leziart/catkin_ws/gazebo_obstacle_debug1_*.txt")
    names2 = glob.glob("/home/leziart/catkin_ws/gazebo_obstacle_debug3_*.txt")
    
    min_x = 1000
    max_x = -1000
    min_y = 1000
    max_y = -1000
    
    colors = ['b', 'g', 'r', 'c', 'm', 'y', "yellowgreen", "slategray", "orange", "maroon"]
    
    fig = plt.figure()
    ax = fig.gca()
    i_color = 0
    for name1 in names1:
        obs1  = (open(name1, "r")).readlines()
        for i_row in range(0,len(obs1)):
            line = obs1[i_row]
            if len(line)==28 :
                x = int(line[16:19])
                y = int(line[21:25])
            elif len(line)==29:
                x = int(line[17:20])
                y = int(line[22:26])
            else:
                x = int(line[18:21])
                y = int(line[23:27])
            rectangle = mpatches.Rectangle((x-0.5,y-0.5), 1, 1, color=colors[i_color])
            if (x < min_x): min_x = x
            if (x > max_x): max_x = x
            if (y < min_y): min_y = y
            if (y > max_y): max_y = y
            ax.add_patch(rectangle)
        i_color += 1
        
    name = glob.glob("/home/leziart/catkin_ws/expanded_mapo.txt")
    obstacles = np.loadtxt(open(name[0], "rb"), delimiter=",")
    for i_row in range(0,obstacles.shape[0]):
        if (i_row%10 == 0): print(i_row)
        
        for i_col in range(0,obstacles.shape[1]):
            if (obstacles[i_row, i_col]==100):
                rectangle = mpatches.Rectangle([i_row-0.5,i_col-0.5], 1, 1, color='k')
                ax.add_patch(rectangle)
        
    ax.set_xlim(min_x - 1, max_x + 1)
    ax.set_ylim(min_y - 1, max_y + 1)
    plt.show()
    
    fig = plt.figure()
    ax = fig.gca()
    i_color = 0
    for name2 in names2:
        obs2  = (open(name2, "r")).readlines()
        for i_row in range(0,len(obs2)):
            line = obs2[i_row]
            if len(line)==28 :
                x = int(line[16:19])
                y = int(line[21:25])
            elif len(line)==29:
                x = int(line[17:20])
                y = int(line[22:26])
            else:
                x = int(line[18:21])
                y = int(line[23:27])
            rectangle = mpatches.Rectangle((x-0.5,y-0.5), 1, 1, color=colors[i_color])
            if (x < min_x): min_x = x
            if (x > max_x): max_x = x
            if (y < min_y): min_y = y
            if (y > max_y): max_y = y
            ax.add_patch(rectangle)
        i_color += 1
    """
    for i_row in range(0,len(obs3)):
        line = obs3[i_row]
        if len(line)==28 :
            rectangle = mpatches.Rectangle((int(line[16:19])-0.5,int(line[21:25])-0.5), 1, 1, color='red')
        else:
            rectangle = mpatches.Rectangle((int(line[17:20])-0.5,int(line[22:26])-0.5), 1, 1, color='red')
        print(rectangle)
        ax.add_patch(rectangle)
    """
    name = glob.glob("/home/leziart/catkin_ws/expanded_mapo.txt")
    obstacles = np.loadtxt(open(name[0], "rb"), delimiter=",")
    for i_row in range(0,obstacles.shape[0]):
        if (i_row%10 == 0): print(i_row)
        
        for i_col in range(0,obstacles.shape[1]):
            if (obstacles[i_row, i_col]==100):
                rectangle = mpatches.Rectangle([i_row-0.5,i_col-0.5], 1, 1, color='k')
                ax.add_patch(rectangle)
                
    ax.set_xlim(min_x - 1, max_x + 1)
    ax.set_ylim(min_y - 1, max_y + 1)
    plt.show()

def disp_debug_occupancy():
    name = glob.glob("/home/leziart/catkin_ws/expanded_mapo.txt")
    fig = plt.figure()
    ax = fig.gca()
    obstacles = np.loadtxt(open(name[0], "rb"), delimiter=",")
    for i_row in range(0,obstacles.shape[0]):
        if (i_row%10 == 0): print(i_row)
        
        for i_col in range(0,obstacles.shape[1]):
            if (obstacles[i_row, i_col]==100):
                rectangle = mpatches.Rectangle([i_row-0.5,i_col-0.5], 1, 1)
                ax.add_patch(rectangle)
    
    x_min, x_max, y_min, y_max = 322 , 336 , 328 , 342




    rectangle = mpatches.Rectangle([x_min-0.5, y_min-0.5], x_max-x_min+1, y_max-y_min+1, fill=False)            
    ax.add_patch(rectangle)
    
    ax.set_xlim(-1, obstacles.shape[0] + 1)
    ax.set_ylim(-1, obstacles.shape[1] + 1)
    plt.show()
    
#disp9()
#disp6()
#disp6bis()
#disp_debug()
#disp_debug_occupancy()
#disp10()
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
