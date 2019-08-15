## File to display different things with Matplotlib by reading txt file containing
# comma-separated data that has been written by the C++ code

import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as anim
import matplotlib.patches as mpatches
import glob
import imageio

## BEZIER FUNCTIONS

def compute_bezier(X,Y):
    n = len(X)
    A = np.vstack((X,Y))
    A = A.transpose()
    
    u = np.ones(n-1)
    mat = 4 * np.eye(n) + np.diag(u,1) + np.diag(u,-1)
    mat[0,n-1] = 1
    mat[n-1,0] = 1
    mat = mat/6
    P = np.dot(np.linalg.inv(mat), A)
    P = np.vstack((P,P[0,:]))
    A = np.vstack((A,A[0,:]))
    B = np.zeros((n,2))
    C = np.zeros((n,2))
    
    for k in range(n):
        B[k,:]=(2*P[k,:]+P[k+1,:])/3;
        C[k,:]=(P[k,:]+2*P[k+1,:])/3;
        
    step=0.01
    t=(np.arange(0,1,step))
    t.reshape((len(t),1))
    s=1-t;
    
    s = s.reshape((len(s),1))
    t = t.reshape((len(t),1))
    
    s3=np.power(s,3);
    s2t=3*np.multiply(np.power(s,2),t)
    t2s=3*np.multiply(np.power(t,2),s)
    t3=np.power(t,3)
    
    return n, A,B,C,s3,s2t,t2s,t3


def border_to_vertices(blob):
    obstacle = []
    blob = np.vstack((blob[-1,:],blob))
    blob = np.vstack((blob,blob[1,:]))
    remove_last = False ## ADD REMOVE LAST TO C++ CODE
    
    for k in range(1,blob.shape[0]-1):
        
        if (blob[k,2]==1):
            if (blob[k,3]==1):
                obstacle.append( [blob[k,0],  blob[k,1]+(1*0.5)])
            elif (blob[k,3]==(-1)):
                obstacle.append( [blob[k,0],  blob[k,1]-(1*0.5)])
            elif (blob[k,4]==(+1)):
                obstacle.append( [blob[k,0]-(1*0.5),  blob[k,1]])
            elif (blob[k,4]==(-1)):
                obstacle.append( [blob[k,0]+(1*0.5),  blob[k,1]])
            else:
                print("Should not happen. Invalid straight line cell." )
                    
            
        elif (blob[k,2]==2):
            if (blob[k,4]==0):
                obstacle.append( [blob[k,0]-(1*0.5),  blob[k,1]])
            elif (blob[k,4]==1):
                obstacle.append( [blob[k,0],  blob[k,1]-(1*0.5)])
            elif (blob[k,4]==2):
                obstacle.append( [blob[k,0]+(1*0.5),  blob[k,1]])
            elif (blob[k,4]==3):
                obstacle.append( [blob[k,0],  blob[k,1]+(1*0.5)])
            else:
                print("Should not happen." )
                
        elif (blob[k,2]==3):
            if (blob[k-1,2] == 2) and (blob[k+1,2] != 2):
                if len(obstacle)>0: obstacle.pop()
                else: remove_last = True  ## ADD REMOVE LAST TO C++ CODE
            
            if (blob[k-1,2] == 1) and (blob[k+1,2] != 3):
               if len(obstacle)>0: obstacle.pop()
               else: remove_last = True  ## ADD REMOVE LAST TO C++ CODE
            
            if (blob[k,0]==10) and ((blob[k,1]==2)): 
                print("PASS")
                print(blob[k-1,:])
                print(blob[k,:])
                print(blob[k+1,:])
            
            if ((blob[k-1,2] == 3) and (blob[k+1,2] == 1)):
                if (blob[k,0]==10) and ((blob[k,1]==2)): print("PASS 1")
                if (blob[k,4]==0):
                    obstacle.append( [blob[k,0],  blob[k,1]-(1*0.5)])
                elif (blob[k,4]==1):
                    obstacle.append( [blob[k,0]+(1*0.5),  blob[k,1]])
                elif (blob[k,4]==2):
                    obstacle.append( [blob[k,0],  blob[k,1]+(1*0.5)])
                elif (blob[k,4]==3):
                    obstacle.append( [blob[k,0]-(1*0.5),  blob[k,1]])
                else:
                    print("Should not happen." )
            elif (blob[k-1,2] == 3) or (blob[k+1,2] == 1) or ((blob[k-1,2] == 1) and (blob[k+1,2] != 3)):
                if (blob[k,0]==10) and ((blob[k,1]==2)): print("PASS 2")
                # Put corner cell
                if (blob[k,4]==0):
                    obstacle.append( [blob[k,0]-(1*0.5),  blob[k,1]-(1*0.5)])
                elif (blob[k,4]==1):
                    obstacle.append( [blob[k,0]+(1*0.5),  blob[k,1]-(1*0.5)])
                elif (blob[k,4]==2):
                    obstacle.append( [blob[k,0]+(1*0.5),  blob[k,1]+(1*0.5)])
                elif (blob[k,4]==3):
                    obstacle.append( [blob[k,0]-(1*0.5),  blob[k,1]+(1*0.5)])
                else:
                    print("Should not happen." )
            elif (blob[k+1,2] == 2) or (blob[k-1,2] == 3):
                if (blob[k,0]==10) and ((blob[k,1]==2)): print("PASS 3")
                if (blob[k,4]==0):
                    obstacle.append( [blob[k,0],  blob[k,1]-(1*0.5)])
                elif (blob[k,4]==1):
                    obstacle.append( [blob[k,0]+(1*0.5),  blob[k,1]])
                elif (blob[k,4]==2):
                    obstacle.append( [blob[k,0],  blob[k,1]+(1*0.5)])
                elif (blob[k,4]==3):
                    obstacle.append( [blob[k,0]-(1*0.5),  blob[k,1]])
                else:
                    print("Should not happen." )
                
        else:
            print("ERROR")
            
    if (remove_last): obstacle.pop()  ## ADD REMOVE LAST TO C++ CODE
    return obstacle

##

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
    names = glob.glob("/home/leziart/Documents/Project_25_06/StreamData/stream_data_0*.txt")
    
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

def disp6ter(): 
    """
    Plot a stream field to display the velocity flow in the workspace
    Each line is formatted as follows "x_pos, y_pos, x_vel, y_vel"
    Use the streamplot function of Matplotlib
    """
    
    attractor = []
    attractor.append([5,-2])
    attractor.append([5,1])
    attractor.append([5,1])
    attractor.append([5,1])
    attractor.append([5,0])
    attractor.append([5,5])
    attractor.append([5,2])
    attractor.append([5,0])
    attractor.append([5,0])
    attractor.append([16,4])
    attractor.append([16,4])
    attractor.append([7.2,-1.5])
    
    margin = 0.25
    limit_dist = 2
    margin_limit = limit_dist - 1#(limit_dist - 1)**0.5
    my_density = 3
    save_figs = False
    
    num = 10
    names = glob.glob("./stream_data_bor*.txt")
    names_normal = glob.glob("D:/Mes documents/Devoirs/MasterThesis/catkin_project/StreamData/stream_data_"+str(num)+"_normal.txt")
    names_bezier = glob.glob("D:/Mes documents/Devoirs/MasterThesis/catkin_project/StreamData/stream_data_"+str(num)+"_bezier.txt")
    names_classic = glob.glob("D:/Mes documents/Devoirs/MasterThesis/catkin_project/StreamData/stream_data_"+str(num)+"_classic.txt")
    names_bor = glob.glob("D:/Mes documents/Devoirs/MasterThesis/catkin_project/StreamData/stream_data_"+str(num)+"_obs.txt")
    names_cells = glob.glob("D:/Mes documents/Devoirs/MasterThesis/catkin_project/StreamData/stream_data_"+str(num)+"_cells.txt")
    
    # # Get data and put in the the correct format
    # for name in names_normal:
    #     entry = np.loadtxt(open(name, "rb"), delimiter=",")
    #     n_size = int(len(entry[:,0])**0.5)
    #     X = np.reshape(entry[:,0], (n_size,n_size)).transpose()
    #     Y = np.reshape(entry[:,1], (n_size,n_size)).transpose()
    #     U = np.reshape(entry[:,2], (n_size,n_size)).transpose()
    #     V = np.reshape(entry[:,3], (n_size,n_size)).transpose()
    # 
    # # Create figure and get axes handle
    # fig, ax = plt.subplots()
    # 
    # # Plot occupied cells
    # for name_cells in names_cells:
    #     obstacles = np.loadtxt(open(name_cells, "rb"), delimiter=",")
    #     for i_row in range(0,obstacles.shape[0]):
    #         rectangle = mpatches.Rectangle([obstacles[i_row,0]-0.5,obstacles[i_row,1]-0.5], 1, 1, color='k')
    #         ax.add_artist(rectangle)
    #             
    # # Plot the borders of obstacles in range
    # borders = np.loadtxt(open(names_bor[0], "rb"), delimiter=",")
    # for i_row in range(0,borders.shape[0]):
    #     if borders[i_row,2] == 1:
    #         if (borders[i_row,3] == 1) and (borders[i_row,4] == 0):
    #             plt.plot([borders[i_row,0]-(0.5-margin), borders[i_row,0]-(0.5-margin)], [borders[i_row,1]-0.5, borders[i_row,1]+0.5], color="r", LineWidth=3)
    #         elif (borders[i_row,3] == -1) and (borders[i_row,4] == 0):
    #             plt.plot([borders[i_row,0]+(0.5-margin), borders[i_row,0]+(0.5-margin)], [borders[i_row,1]-0.5, borders[i_row,1]+0.5], color="r", LineWidth=3)
    #         elif (borders[i_row,3] == 0) and (borders[i_row,4] == 1):
    #             plt.plot([borders[i_row,0]-0.5, borders[i_row,0]+0.5], [borders[i_row,1]-(0.5-margin), borders[i_row,1]-(0.5-margin)], color="r", LineWidth=3)
    #         elif (borders[i_row,3] == 0) and (borders[i_row,4] == -1):
    #             plt.plot([borders[i_row,0]-0.5, borders[i_row,0]+0.5], [borders[i_row,1]+(0.5-margin), borders[i_row,1]+(0.5-margin)], color="r", LineWidth=3)
    #         else:
    #             print("Should not happen")
    #     elif borders[i_row,2] == 2:
    #         if borders[i_row,4] == 0:
    #             arc = mpatches.Arc([borders[i_row,0]-0.5, borders[i_row,1]-0.5],2*margin, 2*margin, 0, 0, 90, LineWidth=3, color="r")
    #         elif borders[i_row,4] == 1:
    #             arc = mpatches.Arc([borders[i_row,0]+0.5, borders[i_row,1]-0.5],2*margin, 2*margin, 0, 90, 180, LineWidth=3, color="r")
    #         elif borders[i_row,4] == 2:
    #             arc = mpatches.Arc([borders[i_row,0]+0.5, borders[i_row,1]+0.5],2*margin, 2*margin, 0, 180, 270, LineWidth=3, color="r")
    #         elif borders[i_row,4] == 3:
    #             arc = mpatches.Arc([borders[i_row,0]-0.5, borders[i_row,1]+0.5],2*margin, 2*margin, 0, 270, 360, LineWidth=3, color="r")
    #         ax.add_artist(arc)
    #     elif borders[i_row,2] == 3:
    #         if borders[i_row,4] == 0:
    #             arc = mpatches.Arc([borders[i_row,0]-0.5, borders[i_row,1]-0.5],2*(1-margin), 2*(1-margin), 0, 0, 90, LineWidth=3, color="r")
    #         elif borders[i_row,4] == 1:
    #             arc = mpatches.Arc([borders[i_row,0]+0.5, borders[i_row,1]-0.5],2*(1-margin), 2*(1-margin), 0, 90, 180, LineWidth=3, color="r")
    #         elif borders[i_row,4] == 2:
    #             arc = mpatches.Arc([borders[i_row,0]+0.5, borders[i_row,1]+0.5],2*(1-margin), 2*(1-margin), 0, 180, 270, LineWidth=3, color="r")
    #         elif borders[i_row,4] == 3:
    #             arc = mpatches.Arc([borders[i_row,0]-0.5, borders[i_row,1]+0.5],2*(1-margin), 2*(1-margin), 0, 270, 360, LineWidth=3, color="r")
    #         ax.add_artist(arc)
    # 
    # # Plot the limits of the area of influence of the obstacle
    # margin = margin_limit + margin
    # for i_row in range(0,borders.shape[0]):
    #     if borders[i_row,2] == 1:
    #         if (borders[i_row,3] == 1) and (borders[i_row,4] == 0):
    #             plt.plot([borders[i_row,0]-(0.5-margin), borders[i_row,0]-(0.5-margin)], [borders[i_row,1]-0.5, borders[i_row,1]+0.5], color="r", LineWidth=3, LineStyle="--", zorder=2)
    #         elif (borders[i_row,3] == -1) and (borders[i_row,4] == 0):
    #             plt.plot([borders[i_row,0]+(0.5-margin), borders[i_row,0]+(0.5-margin)], [borders[i_row,1]-0.5, borders[i_row,1]+0.5], color="r", LineWidth=3, LineStyle="--", zorder=2)
    #         elif (borders[i_row,3] == 0) and (borders[i_row,4] == 1):
    #             plt.plot([borders[i_row,0]-0.5, borders[i_row,0]+0.5], [borders[i_row,1]-(0.5-margin), borders[i_row,1]-(0.5-margin)], color="r", LineWidth=3, LineStyle="--", zorder=2)
    #         elif (borders[i_row,3] == 0) and (borders[i_row,4] == -1):
    #             plt.plot([borders[i_row,0]-0.5, borders[i_row,0]+0.5], [borders[i_row,1]+(0.5-margin), borders[i_row,1]+(0.5-margin)], color="r", LineWidth=3, LineStyle="--", zorder=2)
    #         else:
    #             print("Should not happen")
    #     elif borders[i_row,2] == 2:
    #         if borders[i_row,4] == 0:
    #             arc = mpatches.Arc([borders[i_row,0]-0.5, borders[i_row,1]-0.5],2*margin, 2*margin, 0, 0, 90, LineWidth=3, color="r", LineStyle="--", zorder=2)
    #         elif borders[i_row,4] == 1:
    #             arc = mpatches.Arc([borders[i_row,0]+0.5, borders[i_row,1]-0.5],2*margin, 2*margin, 0, 90, 180, LineWidth=3, color="r", LineStyle="--", zorder=2)
    #         elif borders[i_row,4] == 2:
    #             arc = mpatches.Arc([borders[i_row,0]+0.5, borders[i_row,1]+0.5],2*margin, 2*margin, 0, 180, 270, LineWidth=3, color="r", LineStyle="--", zorder=2)
    #         elif borders[i_row,4] == 3:
    #             arc = mpatches.Arc([borders[i_row,0]-0.5, borders[i_row,1]+0.5],2*margin, 2*margin, 0, 270, 360, LineWidth=3, color="r", LineStyle="--", zorder=2)
    #         ax.add_artist(arc)
    #     elif borders[i_row,2] == 3:
    #         if borders[i_row,4] == 0:
    #             arc = mpatches.Arc([borders[i_row,0]-0.5, borders[i_row,1]-0.5],2*(1-margin), 2*(1-margin), 0, 0, 90, LineWidth=3, color="r", LineStyle="--", zorder=2)
    #         elif borders[i_row,4] == 1:
    #             arc = mpatches.Arc([borders[i_row,0]+0.5, borders[i_row,1]-0.5],2*(1-margin), 2*(1-margin), 0, 90, 180, LineWidth=3, color="r", LineStyle="--", zorder=2)
    #         elif borders[i_row,4] == 2:
    #             arc = mpatches.Arc([borders[i_row,0]+0.5, borders[i_row,1]+0.5],2*(1-margin), 2*(1-margin), 0, 180, 270, LineWidth=3, color="r", LineStyle="--", zorder=2)
    #         elif borders[i_row,4] == 3:
    #             arc = mpatches.Arc([borders[i_row,0]-0.5, borders[i_row,1]+0.5],2*(1-margin), 2*(1-margin), 0, 270, 360, LineWidth=3, color="r", LineStyle="--", zorder=2)
    #         ax.add_artist(arc)
    # 
    # # Add circle around attractor (hand tuned)
    # #ellipse = mpatches.Ellipse([attractor[num][0],attractor[num][1]], 2*5.25, 2*5.25, fill=False, edgecolor="forestgreen", LineWidth=3, zorder=3)
    # #ax.add_artist(ellipse)
    # 
    # # Plot the stream
    # q = ax.streamplot(X, Y, U, V, density=my_density)
    # 
    # # Plot the attractor
    # #ellipse = mpatches.Ellipse([attractor[num][0],attractor[num][1]], 0.3, 0.3, facecolor='forestgreen', edgecolor="k", zorder=5)
    # #ax.add_artist(ellipse)
    #  
    # # Set axis limits and display result
    # min_x = np.min(entry[:,0])
    # max_x = np.max(entry[:,0])
    # min_y = np.min(entry[:,1])
    # max_y = np.max(entry[:,1])
    # 
    # ax.set_xlim(min_x, max_x)
    # ax.set_ylim(min_y, max_y)
    # #ax.set_xlim(300, 390)
    # #ax.set_ylim(280, 370)
    # ax.set_aspect("equal")
    # plt.show()
    # if save_figs:
    #     fig.savefig("/home/leziart/Pictures/Normal_VS_Bezier/normal_"+str(num)+".png", dpi=300)
    #     #fig.savefig("/home/leziart/Pictures/Normal_VS_Bezier/normal_"+str(num)+".svg", format='svg', dpi=600)
    #     fig.savefig("/home/leziart/Pictures/Normal_VS_Bezier/normal_"+str(num)+".eps", format='eps')
    # 
    # Get data and put in the the correct format
    for name in names_bezier:
        entry = np.loadtxt(open(name, "rb"), delimiter=",")
        norm_vec = (entry[:,2]**2 + entry[:,3]**2)**0.5
        norm_vec[norm_vec==0] = 1
        entry[:,2] = entry[:,2] / norm_vec
        entry[:,3] = entry[:,3] / norm_vec
        n_size = int(len(entry[:,0])**0.5)
        X = np.reshape(entry[:,0], (n_size,n_size)).transpose()
        Y = np.reshape(entry[:,1], (n_size,n_size)).transpose()
        U = np.reshape(entry[:,2], (n_size,n_size)).transpose()
        V = np.reshape(entry[:,3], (n_size,n_size)).transpose()
        
        # X = X[0:30,0:30]
        # Y = Y[0:30,0:30]
        # U = U[0:30,0:30]
        # V = V[0:30,0:30]
        
        # X = entry[:,0]
        # Y = entry[:,1]
        # U = entry[:,2]
        # V = entry[:,3]
        # M = np.hypot(U, V)
    # Create figure and get axes handle
    fig, ax = plt.subplots()
    
    # Plot occupied cells
    for name_cells in names_cells:
        obstacles = np.loadtxt(open(name_cells, "rb"), delimiter=",")
        for i_row in range(0,obstacles.shape[0]):
            rectangle = mpatches.Rectangle([obstacles[i_row,0]-0.5,obstacles[i_row,1]-0.5], 1, 1, color='k')
            ax.add_artist(rectangle)
                    
    data = np.loadtxt(open(names_bor[0], "rb"), delimiter=",")
    
    if data.shape[1]==6:
        for i in np.unique(data[:,5]):
            output_data = border_to_vertices(data[data[:,5]==i,0:5])
            output_data = np.array(output_data)
            
            X_bez = output_data[:,0]
            Y_bez = output_data[:,1]
            
            n, A,B,C,s3,s2t,t2s,t3 = compute_bezier(X_bez,Y_bez)
            
            for k in range(n):  
                a=A[k,:]
                b=B[k,:]
                c=C[k,:]
                d=A[k+1,:]
                a = a.reshape((len(a),1))
                b = b.reshape((len(b),1))
                c = c.reshape((len(c),1))
                d = d.reshape((len(d),1))
                bez= s3*a.transpose() + s2t * b.transpose() + t2s*c.transpose() + t3*d.transpose()
                #plt.plot((bez[:,0]).transpose(),(bez[:,1]).transpose(), linewidth=4, color='red')
    else:
        output_data = border_to_vertices(data)
        output_data = np.array(output_data)
        
        X_bez = output_data[:,0]
        Y_bez = output_data[:,1]
        
        n, A,B,C,s3,s2t,t2s,t3 = compute_bezier(X_bez,Y_bez)
        
        for k in range(n):  
            a=A[k,:]
            b=B[k,:]
            c=C[k,:]
            d=A[k+1,:]
            a = a.reshape((len(a),1))
            b = b.reshape((len(b),1))
            c = c.reshape((len(c),1))
            d = d.reshape((len(d),1))
            bez= s3*a.transpose() + s2t * b.transpose() + t2s*c.transpose() + t3*d.transpose()
            #plt.plot((bez[:,0]).transpose(),(bez[:,1]).transpose(), linewidth=4, color='red')
    
    names_pts_bezier = glob.glob("D:/Mes documents/Devoirs/MasterThesis/catkin_project/StreamData/stream_data_"+str(num)+"_border_bezier_*.txt")
    
    for name in names_pts_bezier:
        border_bezier = np.loadtxt(open(name, "rb"), delimiter=",")
    
        N = int(border_bezier.shape[0] / 2)
        for i in range(border_bezier.shape[1]):
            plt.plot(border_bezier[0:100,i],border_bezier[100:,i],linewidth=4, color='red')
        
    # Plot the stream
    q = ax.streamplot(X, Y, U, V, density=my_density)
    #q = ax.quiver(X, Y, U, V, facecolor="dodgerblue")
    
    # Plot the attractor
    #ellipse = mpatches.Ellipse([attractor[num][0],attractor[num][1]], 0.2, 0.2, facecolor='forestgreen', edgecolor="k", zorder=5)
    #ax.add_artist(ellipse)
    ellipse = mpatches.Ellipse([0, 0], 0.5, 0.5, facecolor='forestgreen', edgecolor="k", zorder=5)
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
    ax.set_aspect("equal")
    #♣fig.savefig("D:/Mes documents/Devoirs/MasterThesis/catkin_project/StreamData/bezier_"+str(num)+".eps", format='eps', bbox_inches = 'tight', pad_inches = 0)
    
    manager = plt.get_current_fig_manager()
    manager.window.showMaximized()
    plt.show()
    fig.savefig("D:/Mes documents/Devoirs/MasterThesis/catkin_project/StreamData/bezier_flow.eps", format='eps', bbox_inches = 'tight', pad_inches = 0)
    
    if save_figs:
        fig.savefig("/home/leziart/Pictures/Normal_VS_Bezier/bezier_"+str(num)+".png", dpi=300)
        #fig.savefig("/home/leziart/Pictures/Normal_VS_Bezier/bezier_"+str(num)+".svg", format='svg', dpi=600)
        fig.savefig("/home/leziart/Pictures/Normal_VS_Bezier/bezier_"+str(num)+".eps", format='eps')
    
    if (num >= 6):
        ref_point = []
        ref_point.append([5,4.5])
        ref_point.append([4.5,7.5])
        ref_point.append([4.5,7.5])
        ref_point.append([10,10])
        
        # Get data and put in the the correct format
        for name in names_classic:
            entry = np.loadtxt(open(name, "rb"), delimiter=",")
            n_size = int(len(entry[:,0])**0.5)
            X = np.reshape(entry[:,0], (n_size,n_size)).transpose()
            Y = np.reshape(entry[:,1], (n_size,n_size)).transpose()
            U = np.reshape(entry[:,2], (n_size,n_size)).transpose()
            V = np.reshape(entry[:,3], (n_size,n_size)).transpose()
        
        # Create figure and get axes handle
        fig, ax = plt.subplots()
        
        # Plot occupied cells
        for name_cells in names_cells:
            obstacles = np.loadtxt(open(name_cells, "rb"), delimiter=",")
            for i_row in range(0,obstacles.shape[0]):
                rectangle = mpatches.Rectangle([obstacles[i_row,0]-0.5,obstacles[i_row,1]-0.5], 1, 1, color='k')
                ax.add_artist(rectangle)
                    
        # Plot the borders of obstacles in range
        borders = np.loadtxt(open(names_bor[0], "rb"), delimiter=",")
        for i_row in range(0,borders.shape[0]):
            if borders[i_row,2] == 1:
                if (borders[i_row,3] == 1) and (borders[i_row,4] == 0):
                    plt.plot([borders[i_row,0]-(0.5-margin), borders[i_row,0]-(0.5-margin)], [borders[i_row,1]-0.5, borders[i_row,1]+0.5], color="r", LineWidth=3)
                elif (borders[i_row,3] == -1) and (borders[i_row,4] == 0):
                    plt.plot([borders[i_row,0]+(0.5-margin), borders[i_row,0]+(0.5-margin)], [borders[i_row,1]-0.5, borders[i_row,1]+0.5], color="r", LineWidth=3)
                elif (borders[i_row,3] == 0) and (borders[i_row,4] == 1):
                    plt.plot([borders[i_row,0]-0.5, borders[i_row,0]+0.5], [borders[i_row,1]-(0.5-margin), borders[i_row,1]-(0.5-margin)], color="r", LineWidth=3)
                elif (borders[i_row,3] == 0) and (borders[i_row,4] == -1):
                    plt.plot([borders[i_row,0]-0.5, borders[i_row,0]+0.5], [borders[i_row,1]+(0.5-margin), borders[i_row,1]+(0.5-margin)], color="r", LineWidth=3)
                else:
                    print("Should not happen")
            elif borders[i_row,2] == 2:
                if borders[i_row,4] == 0:
                    arc = mpatches.Arc([borders[i_row,0]-0.5, borders[i_row,1]-0.5],2*margin, 2*margin, 0, 0, 90, LineWidth=3, color="r")
                elif borders[i_row,4] == 1:
                    arc = mpatches.Arc([borders[i_row,0]+0.5, borders[i_row,1]-0.5],2*margin, 2*margin, 0, 90, 180, LineWidth=3, color="r")
                elif borders[i_row,4] == 2:
                    arc = mpatches.Arc([borders[i_row,0]+0.5, borders[i_row,1]+0.5],2*margin, 2*margin, 0, 180, 270, LineWidth=3, color="r")
                elif borders[i_row,4] == 3:
                    arc = mpatches.Arc([borders[i_row,0]-0.5, borders[i_row,1]+0.5],2*margin, 2*margin, 0, 270, 360, LineWidth=3, color="r")
                ax.add_artist(arc)
            elif borders[i_row,2] == 3:
                if borders[i_row,4] == 0:
                    arc = mpatches.Arc([borders[i_row,0]-0.5, borders[i_row,1]-0.5],2*margin, 2*margin, 0, 0, 90, LineWidth=3, color="r")
                elif borders[i_row,4] == 1:
                    arc = mpatches.Arc([borders[i_row,0]+0.5, borders[i_row,1]-0.5],2*margin, 2*margin, 0, 90, 180, LineWidth=3, color="r")
                elif borders[i_row,4] == 2:
                    arc = mpatches.Arc([borders[i_row,0]+0.5, borders[i_row,1]+0.5],2*margin, 2*margin, 0, 180, 270, LineWidth=3, color="r")
                elif borders[i_row,4] == 3:
                    arc = mpatches.Arc([borders[i_row,0]-0.5, borders[i_row,1]+0.5],2*margin, 2*margin, 0, 270, 360, LineWidth=3, color="r")
                ax.add_artist(arc)
        
        # Plot the stream
        q = ax.streamplot(X, Y, U, V, density=my_density)
        
        # Plot the attractor
        ellipse = mpatches.Ellipse([attractor[num][0],attractor[num][1]], 0.2, 0.2, facecolor='forestgreen', edgecolor="k", zorder=5)
        ax.add_artist(ellipse)
        
        # Plot the reference point
        ellipse = mpatches.Ellipse([ref_point[num-6][0],ref_point[num-6][1]], 0.3, 0.3, facecolor='cyan', edgecolor="white", zorder=5)
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
        ax.set_aspect("equal")
        plt.show()
        if save_figs:
            fig.savefig("/home/leziart/Pictures/Normal_VS_Bezier/classic_"+str(num)+".png", dpi=300)
            fig.savefig("/home/leziart/Pictures/Normal_VS_Bezier/classic_"+str(num)+".eps", format='eps')

from matplotlib.colors import BoundaryNorm
from matplotlib.ticker import MaxNLocator

def get_D_T(X,Y,U,V,n_size,a_x,a_y,h):
    resolution = X[0][1]-X[0][0]
    
    T_matrix = np.zeros([n_size,n_size])
    D_matrix = np.zeros([n_size,n_size])
    
    for i in range(n_size):
        print(str(i)+"/"+str(n_size))
        for j in range(n_size):
            x = X[i][j]
            y = Y[i][j]
            T = 0
            D = 0
            
            max_T = 3000
            i_loop = 0
            while (T<max_T) and (np.sqrt(np.power(x-a_x,2)+np.power(y-a_y,2)) > 0.3) : 
                i_loop += 1
                
                i_x = int(np.floor((x - X[0][0])/resolution))
                i_y = int(np.floor((y - Y[0][0])/resolution))
                
                v_x = np.mean(V[i_x:(i_x+2),i_y:(i_y+2)])
                v_y = np.mean(U[i_x:(i_x+2),i_y:(i_y+2)])
                
                x += v_x * h
                y += v_y * h
                T += h
                D += np.sqrt(np.power(v_x * h,2)+np.power(v_y * h,2))
                
                if (v_x==0) and (v_y==0): T = max_T
    
                if np.isnan(D):
                     a = 1
    
            T_matrix[i,j] = T
            if (T>=max_T) and (i_loop<=5):
                D_matrix[i,j] = -42
                T_matrix[i,j] = -42
            elif (T>=max_T):
                D_matrix[i,j] = -43
                T_matrix[i,j] = -43
            else:
                D_matrix[i,j] = D
        
    
    return D_matrix, T_matrix
    
def disp_metrics(): 
    """
    Plot a stream field to display the velocity flow in the workspace
    Each line is formatted as follows "x_pos, y_pos, x_vel, y_vel"
    Use the streamplot function of Matplotlib
    """
    
    a_x = 2
    a_y = 5
    
    h = 0.1
    margin = 0.5
    my_density = 3
    
    num = 0
    names = glob.glob("./stream_data_bor*.txt")
    names_normal = glob.glob("/home/leziart/Documents/Project_25_06/StreamData/stream_data_"+str(num)+"_normal.txt")
    names_bezier = glob.glob("/home/leziart/Documents/Project_25_06/StreamData/stream_data_"+str(num)+"_bezier.txt")
    names_classic = glob.glob("/home/leziart/Documents/Project_25_06/StreamData/stream_data_"+str(num)+"_classic.txt")
    names_bor = glob.glob("/home/leziart/Documents/Project_25_06/StreamData/stream_data_"+str(num)+"_obs.txt")
    names_cells = glob.glob("/home/leziart/Documents/Project_25_06/StreamData/stream_data_"+str(num)+"_cells.txt")
    
    print("### Starting normal ###")
    # Get data and put in the the correct format
    for name in names_normal:
        entry = np.loadtxt(open(name, "rb"), delimiter=",")
        n_size = int(len(entry[:,0])**0.5)
        X = np.reshape(entry[:,0], (n_size,n_size)).transpose()
        Y = np.reshape(entry[:,1], (n_size,n_size)).transpose()
        U = np.reshape(entry[:,2], (n_size,n_size)).transpose()
        V = np.reshape(entry[:,3], (n_size,n_size)).transpose()
        
    U = np.nan_to_num(U)
    V = np.nan_to_num(V)
    
    
    # resolution = X[0][1]-X[0][0]
    # 
    # T_matrix = np.zeros([n_size,n_size])
    # D_matrix = np.zeros([n_size,n_size])
    # 
    # for i in range(n_size):
    #     print(str(i)+"/"+str(n_size))
    #     for j in range(n_size):
    #         x = X[i][j]
    #         y = Y[i][j]
    #         T = 0
    #         D = 0
    #         
    #         #list_x = [x]
    #         #list_y = [y]
    #         
    #         max_T = 1000
    #         i_loop = 0
    #         while (T<max_T) and (np.sqrt(np.power(x-a_x,2)+np.power(y-a_y,2)) > 0.3) : 
    #             i_loop += 1
    #             
    #             i_x = int(np.floor((x - X[0][0])/resolution))
    #             i_y = int(np.floor((y - Y[0][0])/resolution))
    #             
    #             v_x = np.mean(V[i_x:i_x+2,i_y:i_y+2])
    #             v_y = np.mean(U[i_x:i_x+2,i_y:i_y+2])
    #             
    #             if (np.isnan(v_x)): v_x = 0
    #             x += v_x * h
    #             y += v_y * h
    #             T += h
    #             D += np.sqrt(np.power(v_x * h,2)+np.power(v_y * h,2))
    #             #list_x.append(x)
    #             #list_y.append(y)
    #             
    #             if (v_x==0) and (v_y==0): T = max_T
    #             
    #             if np.isnan(D):
    #                 a = 1
    # 
    #         T_matrix[i,j] = T
    #         if (T>=400) and (i_loop<=5):
    #             D_matrix[i,j] = -42
    #         elif (T>=400):
    #             D_matrix[i,j] = -43
    #         else:
    #             D_matrix[i,j] = D
            
        # Create figure and get axes handle
        #fig, ax = plt.subplots()
        
        # Plot the stream
        #q = ax.streamplot(X, Y, U, V, density=my_density)
        #ax.plot_surface(X, Y, Z)
        
        # Plot the reference point
        # for k in range(len(list_x)):
        #     ellipse = mpatches.Ellipse([list_y[k],list_x[k]], 0.2, 0.2, facecolor='cyan', zorder=5)
        #     ax.add_artist(ellipse)
        
        #plt.plot(list_x,list_y,"x",linewidth=3)
        #plt.title("Time for completion: " + str(T))
        #plt.show()
        
    #D_matrix[D_matrix==(-42)] = np.nanmax(D_matrix)
    
    D_matrix, T_matrix = get_D_T(X,Y,U,V,n_size,a_x,a_y,h)
    
    np.save("/home/leziart/Documents/Metrics/normal_D_"+str(num)+".npy", D_matrix)
    np.save("/home/leziart/Documents/Metrics/normal_T_"+str(num)+".npy", T_matrix)
    
    print("### Starting bezier ###")
    # Get data and put in the the correct format
    for name in names_bezier:
        entry = np.loadtxt(open(name, "rb"), delimiter=",")
        n_size = int(len(entry[:,0])**0.5)
        X = np.reshape(entry[:,0], (n_size,n_size)).transpose()
        Y = np.reshape(entry[:,1], (n_size,n_size)).transpose()
        U = np.reshape(entry[:,2], (n_size,n_size)).transpose()
        V = np.reshape(entry[:,3], (n_size,n_size)).transpose()
    
    U = np.nan_to_num(U)
    V = np.nan_to_num(V)
    # resolution = X[0][1]-X[0][0]
    # 
    # T_matrix = np.zeros([n_size,n_size])
    # D_matrix = np.zeros([n_size,n_size])
    # 
    # for i in range(n_size):
    #     print(str(i)+"/"+str(n_size))
    #     for j in range(n_size):
    #         x = X[i][j]
    #         y = Y[i][j]
    #         T = 0
    #         D = 0
    #         
    #         max_T = 1000
    #         while (T<max_T) and (np.sqrt(np.power(x-a_x,2)+np.power(y-a_y,2)) > 0.3) : 
    #             i_x = int(np.floor((x - X[0][0])/resolution))
    #             i_y = int(np.floor((y - Y[0][0])/resolution))
    #             
    #             v_x = np.mean(V[i_x:i_x+2,i_y:i_y+2])
    #             v_y = np.mean(U[i_x:i_x+2,i_y:i_y+2])
    #             
    #             x += v_x * h
    #             y += v_y * h
    #             T += h
    #             D += np.sqrt(np.power(v_x * h,2)+np.power(v_y * h,2))

   ##               
    #             if (v_x==0) and (v_y==0): T = max_T
    # 
    #         T_matrix[i,j] = T
    #         if (T>=400) and (i_loop<=5):
    #             D_matrix[i,j] = -42
    #         elif (T>=400):
    #             D_matrix[i,j] = -43
    #         else:
    #             D_matrix[i,j] = D
            
        
    # D_matrix[D_matrix==(-42)] = np.nanmax(D_matrix)
    
    D_matrix, T_matrix = get_D_T(X,Y,U,V,n_size,a_x,a_y,h)
    
    np.save("/home/leziart/Documents/Metrics/bezier_D_"+str(num)+".npy", D_matrix)
    np.save("/home/leziart/Documents/Metrics/bezier_T_"+str(num)+".npy", T_matrix)
    
    print("### Files saved ###")
    # fig, (ax0, ax1) = plt.subplots(nrows=2)
    # 
    # levels = MaxNLocator(nbins=30).tick_values(T_matrix.min(), T_matrix.max()) 
    # cmap = plt.get_cmap('PiYG')
    # norm = BoundaryNorm(levels, ncolors=cmap.N, clip=True)
    # im = ax0.pcolormesh(Y, X, T_matrix, cmap=cmap, norm=norm)
    # fig.colorbar(im, ax=ax0)
    # ax0.set_title('Time for completion')
    # 
    # levels = MaxNLocator(nbins=30).tick_values(D.min(), D.max()) 
    # cmap = plt.get_cmap('PiYG')
    # norm = BoundaryNorm(levels, ncolors=cmap.N, clip=True)
    # im = ax1.pcolormesh(Y, X, D_matrix, cmap=cmap, norm=norm)
    # fig.colorbar(im, ax=ax1)
    # ax1.set_title('Distance for completion')
    # 
    # # fig, ax = plt.subplots()
    # # ax.plot_surface(X, Y, T)
    # # plt.title("Time for completion")
    # # plt.show()
    # # 
    # # fig, ax = plt.subplots()
    # # ax.plot_surface(X, Y, D)
    # # plt.title("Distance for completion")
    # 
    # fig.tight_layout()
    # plt.show()

def disp_metrics_bis(): 
    """
    Plot a stream field to display the velocity flow in the workspace
    Each line is formatted as follows "x_pos, y_pos, x_vel, y_vel"
    Use the streamplot function of Matplotlib
    """
    
    a_x = 0
    a_y = 5
    
    h = 0.1
    margin = 0.5
    my_density = 3
    
    num = 9
    names = glob.glob("./stream_data_bor*.txt")
    names_normal = glob.glob("/home/leziart/Documents/Project_25_06/StreamData/stream_data_"+str(num)+"_normal.txt")
    names_bezier = glob.glob("/home/leziart/Documents/Project_25_06/StreamData/stream_data_"+str(num)+"_bezier.txt")
    names_classic = glob.glob("/home/leziart/Documents/Project_25_06/StreamData/stream_data_"+str(num)+"_classic.txt")
    names_bor = glob.glob("/home/leziart/Documents/Project_25_06/StreamData/stream_data_"+str(num)+"_obs.txt")
    names_cells = glob.glob("/home/leziart/Documents/Project_25_06/StreamData/stream_data_"+str(num)+"_cells.txt")
    
    print("### Starting normal ###")
    # Get data and put in the the correct format
    for name in names_normal:
        entry = np.loadtxt(open(name, "rb"), delimiter=",")
        n_size = int(len(entry[:,0])**0.5)
        X = np.reshape(entry[:,0], (n_size,n_size)).transpose()
        Y = np.reshape(entry[:,1], (n_size,n_size)).transpose()
        U = np.reshape(entry[:,2], (n_size,n_size)).transpose()
        V = np.reshape(entry[:,3], (n_size,n_size)).transpose()
        U = np.nan_to_num(U)
        V = np.nan_to_num(V)
        D_matrix, T_matrix = get_D_T(X,Y,U,V,n_size,a_x,a_y,h)
        np.save("/home/leziart/Documents/Metrics/normal_D_"+str(num)+".npy", D_matrix)
        np.save("/home/leziart/Documents/Metrics/normal_T_"+str(num)+".npy", T_matrix)
    
    print("### Starting bezier ###")
    # Get data and put in the the correct format
    for name in names_bezier:
        entry = np.loadtxt(open(name, "rb"), delimiter=",")
        n_size = int(len(entry[:,0])**0.5)
        X = np.reshape(entry[:,0], (n_size,n_size)).transpose()
        Y = np.reshape(entry[:,1], (n_size,n_size)).transpose()
        U = np.reshape(entry[:,2], (n_size,n_size)).transpose()
        V = np.reshape(entry[:,3], (n_size,n_size)).transpose()
        U = np.nan_to_num(U)
        V = np.nan_to_num(V)
        D_matrix, T_matrix = get_D_T(X,Y,U,V,n_size,a_x,a_y,h)
        np.save("/home/leziart/Documents/Metrics/bezier_D_"+str(num)+".npy", D_matrix)
        np.save("/home/leziart/Documents/Metrics/bezier_T_"+str(num)+".npy", T_matrix)
    
    print("### Starting classic ###")
    # Get data and put in the the correct format
    for name in names_classic:
        entry = np.loadtxt(open(name, "rb"), delimiter=",")
        n_size = int(len(entry[:,0])**0.5)
        X = np.reshape(entry[:,0], (n_size,n_size)).transpose()
        Y = np.reshape(entry[:,1], (n_size,n_size)).transpose()
        U = np.reshape(entry[:,2], (n_size,n_size)).transpose()
        V = np.reshape(entry[:,3], (n_size,n_size)).transpose()
        U = np.nan_to_num(U)
        V = np.nan_to_num(V)
        D_matrix, T_matrix = get_D_T(X,Y,U,V,n_size,a_x,a_y,h)
        np.save("/home/leziart/Documents/Metrics/classic_D_"+str(num)+".npy", D_matrix)
        np.save("/home/leziart/Documents/Metrics/classic_T_"+str(num)+".npy", T_matrix)
    
    print("### Files saved ###")

    
def disp_metrics_plots():
    num = 7
    for i_num in range(1):
        names = glob.glob("./stream_data_bor*.txt")
        names_normal = glob.glob("/home/leziart/Documents/Project_25_06/StreamData/stream_data_"+str(num)+"_normal.txt")
        names_bezier = glob.glob("/home/leziart/Documents/Project_25_06/StreamData/stream_data_"+str(num)+"_bezier.txt")
        names_classic = glob.glob("/home/leziart/Documents/Project_25_06/StreamData/stream_data_"+str(num)+"_classic.txt")
        names_bor = glob.glob("/home/leziart/Documents/Project_25_06/StreamData/stream_data_"+str(num)+"_obs.txt")
        names_cells = glob.glob("/home/leziart/Documents/Project_25_06/StreamData/stream_data_"+str(num)+"_cells.txt")
        
        # Get data and put in the the correct format
        for name in names_normal:
            entry = np.loadtxt(open(name, "rb"), delimiter=",")
            n_size = int(len(entry[:,0])**0.5)
            X = np.reshape(entry[:,0], (n_size,n_size)).transpose()
            Y = np.reshape(entry[:,1], (n_size,n_size)).transpose()
            U = np.reshape(entry[:,2], (n_size,n_size)).transpose()
            V = np.reshape(entry[:,3], (n_size,n_size)).transpose()
            
        D_normal = np.load("/home/leziart/Documents/Metrics/normal_D_"+str(num)+".npy")
        T_normal = np.load("/home/leziart/Documents/Metrics/normal_T_"+str(num)+".npy")

        D_bezier = np.load("/home/leziart/Documents/Metrics/bezier_D_"+str(num)+".npy")
        T_bezier = np.load("/home/leziart/Documents/Metrics/bezier_T_"+str(num)+".npy")
             
        D_classic = np.load("/home/leziart/Documents/Metrics/classic_D_"+str(num)+".npy")
        T_classic = np.load("/home/leziart/Documents/Metrics/classic_T_"+str(num)+".npy")
        
        I_D = np.argwhere(np.isnan(D_classic))
        for i_pair in range(I_D.shape[0]):
            pair = I_D[i_pair,:]
            pair_temp = np.copy(pair)
            while np.isnan(D_classic[pair_temp[0],pair_temp[1]]):
                pair_temp[1] -= 1
            D_classic[pair[0],pair[1]] = D_classic[pair_temp[0],pair_temp[1]]
            
        D_classic[np.argwhere(np.isnan(D_classic))] = 0
        T_classic[np.argwhere(np.isnan(T_classic))] = 0
        
        T_normal[T_normal==(-42)] = 0.0
        T_normal[T_normal==(-43)] = 0.0
        T_bezier[T_bezier==(-42)] = 0.0
        T_bezier[T_bezier==(-43)] = 0.0
        T_classic[T_classic==(-42)] = 0.0
        T_classic[T_classic==(-43)] = 0.0
        
        fig, (ax2, ax0, ax1) = plt.subplots(nrows=3)
        
        levels = MaxNLocator(nbins=30).tick_values(np.min([np.min(T_normal),np.min(T_bezier),np.min(T_classic)]), np.max([np.max(T_normal),np.max(T_bezier),np.max(T_classic)])) 
        #levels = np.linspace(np.min([np.min(T_normal),np.min(T_bezier)]), np.max([np.max(T_normal),np.max(T_bezier)]), 15)
        cmap = plt.get_cmap('PiYG')
        norm = BoundaryNorm(levels, ncolors=cmap.N, clip=True)
             
        im = ax0.pcolormesh(Y, X, T_normal, cmap=cmap, norm=norm)
        fig.colorbar(im, ax=ax0)
        ax0.set_title('Time for completion normal')
        
        resolution = X[0][1]-X[0][0]
        I_D = np.argwhere(D_normal==(-42))
        D_normal[D_normal==(-42)] = np.max([D_normal.max(),D_bezier.max(),np.max(D_classic)])
        D_normal[D_normal==(-43)] = np.max([D_normal.max(),D_bezier.max(),np.max(D_classic)])
        # for i in range(I_D.shape[0]):
        #     rectangle = mpatches.Rectangle([X[I_D[i,1],I_D[i,0]]-resolution*0.5,Y[I_D[i,1],I_D[i,0]]-resolution*0.5], resolution, resolution, color='k')
        #     ax0.add_artist(rectangle)
                
                
        im = ax1.pcolormesh(Y, X, T_bezier, cmap=cmap, norm=norm)
        fig.colorbar(im, ax=ax1)
        ax1.set_title('Time for completion bezier')
        
        resolution = X[0][1]-X[0][0]
        I_D = np.argwhere(D_bezier==(-42))
        D_bezier[D_bezier==(-42)] = np.max([D_normal.max(),D_bezier.max(),np.max(D_classic)])
        D_bezier[D_bezier==(-43)] = np.max([D_normal.max(),D_bezier.max(),np.max(D_classic)])
        # for i in range(I_D.shape[0]):
        #     rectangle = mpatches.Rectangle([X[I_D[i,1],I_D[i,0]]-resolution*0.5,Y[I_D[i,1],I_D[i,0]]-resolution*0.5], resolution, resolution, color='k')
        #     ax1.add_artist(rectangle)
        
        im = ax2.pcolormesh(Y, X, T_classic, cmap=cmap, norm=norm)
        fig.colorbar(im, ax=ax2)
        ax2.set_title('Time for completion original')
        
        resolution = X[0][1]-X[0][0]
        I_D = np.argwhere(D_classic==(-42))
        D_classic[D_classic==(-42)] = np.max([D_normal.max(),D_bezier.max(),np.max(D_classic)])
        D_classic[D_classic==(-43)] = np.max([D_normal.max(),D_bezier.max(),np.max(D_classic)])
        
        fig.tight_layout()
        plt.show()
        
        
        
        fig, (ax2, ax0, ax1) = plt.subplots(nrows=3)
        
        levels = MaxNLocator(nbins=30).tick_values(np.min([D_normal.min(),D_bezier.min(),np.min(D_classic)]), np.max([D_normal.max(),D_bezier.max(),np.max(D_classic)])) 
        #levels = np.linspace(np.min(np.min(D_normal),np.min(D_bezier)), np.max(np.max(D_normal),np.max(D_bezier)), 15)
        cmap = plt.get_cmap('PiYG')
        norm = BoundaryNorm(levels, ncolors=cmap.N, clip=True)
             
        im = ax0.pcolormesh(Y, X, D_normal, cmap=cmap, norm=norm)
        fig.colorbar(im, ax=ax0)
        ax0.set_title('Distance for completion normal')
        
        im = ax1.pcolormesh(Y, X, D_bezier, cmap=cmap, norm=norm)
        fig.colorbar(im, ax=ax1)
        ax1.set_title('Distance for completion bezier')
        
        im = ax2.pcolormesh(Y, X, D_classic, cmap=cmap, norm=norm)
        fig.colorbar(im, ax=ax2)
        ax2.set_title('Distance for completion original')
        
        fig.tight_layout()
        plt.show()
        
        fig, ax = plt.subplots()
        
        
        levels = MaxNLocator(nbins=30).tick_values(np.min([D_normal.min(),D_bezier.min(),np.min(D_classic)]), np.max([D_normal.max(),D_bezier.max(),np.max(D_classic)])) 
        #levels = np.linspace(np.min(np.min(D_normal),np.min(D_bezier)), np.max(np.max(D_normal),np.max(D_bezier)), 15)
        cmap = plt.get_cmap('PiYG')
        norm = BoundaryNorm(levels, ncolors=cmap.N, clip=True)
           
        fig, ax0 = plt.subplots()
        im = ax0.pcolormesh(Y, X, D_normal, cmap=cmap, norm=norm)
        fig.colorbar(im, ax=ax0)
        ax0.set_title('Distance for completion normal')
        plt.show()
        fig.savefig("/home/leziart/Documents/Metrics_Result/distance_"+str(num)+"_normal.png", dpi=200)
        
        fig, ax1 = plt.subplots()
        im = ax1.pcolormesh(Y, X, D_bezier, cmap=cmap, norm=norm)
        fig.colorbar(im, ax=ax1)
        ax1.set_title('Distance for completion bezier')
        plt.show()
        fig.savefig("/home/leziart/Documents/Metrics_Result/distance_"+str(num)+"_bezier.png", dpi=200)
        
        fig, ax2 = plt.subplots()
        im = ax2.pcolormesh(Y, X, D_classic, cmap=cmap, norm=norm)
        fig.colorbar(im, ax=ax2)
        ax2.set_title('Distance for completion original')
        plt.show()
        fig.savefig("/home/leziart/Documents/Metrics_Result/distance_"+str(num)+"_classic.png", dpi=200)
        
        
    
    
    
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

def disp_trajectory_comparison(): 
    """
    Plot the trajectory of the robot in circle space considering the initial space
    as well as the trajectory of the robot in the circle space entirely
    """
    import matplotlib.patches as mpatches
    import matplotlib.lines as mlines
    
    attractor = [0,5]
    
    margin = 0.25
    limit_dist = 3
    margin_limit = (limit_dist - 1)**0.5
    my_density = 3
    save_figs = False
    
    num = 1
    names = glob.glob("./stream_data_bor*.txt")
    names_normal = glob.glob("D:/Mes documents/Devoirs/MasterThesis/catkin_project/TrajectoryComparisonData/traj_data_"+str(num)+"_normal.txt")
    names_corrected = glob.glob("D:/Mes documents/Devoirs/MasterThesis/catkin_project/TrajectoryComparisonData/traj_data_"+str(num)+"_corrected.txt")
    names_corrected_circle = glob.glob("D:/Mes documents/Devoirs/MasterThesis/catkin_project/TrajectoryComparisonData/traj_data_"+str(num)+"_corrected_circle.txt")
    names_circle = glob.glob("D:/Mes documents/Devoirs/MasterThesis/catkin_project/TrajectoryComparisonData/traj_data_"+str(num)+"_circle.txt")
    names_only_circle = glob.glob("D:/Mes documents/Devoirs/MasterThesis/catkin_project/TrajectoryComparisonData/traj_data_"+str(num)+"_only_circle.txt")
    names_bor = glob.glob("D:/Mes documents/Devoirs/MasterThesis/catkin_project/TrajectoryComparisonData/traj_data_"+str(num)+"_obs.txt")
    names_cells = glob.glob("D:/Mes documents/Devoirs/MasterThesis/catkin_project/TrajectoryComparisonData/traj_data_"+str(num)+"_cells.txt")
    names_params = glob.glob("D:/Mes documents/Devoirs/MasterThesis/catkin_project/TrajectoryComparisonData/traj_data_"+str(num)+"_params.txt")
    #names_corrected_circle_formula = glob.glob("D:/Mes documents/Devoirs/MasterThesis/catkin_project/TrajectoryComparisonData/traj_data_"+str(num)+"_corrected_circle_formula.txt")
    
    entry_normal = np.loadtxt(open(names_normal[0], "rb"), delimiter=",")
    entry_corrected = np.loadtxt(open(names_corrected[0], "rb"), delimiter=",")
    entry_corrected_circle = np.loadtxt(open(names_corrected_circle[0], "rb"), delimiter=",")
    entry_circle = np.loadtxt(open(names_circle[0], "rb"), delimiter=",")
    entry_only_circle = np.loadtxt(open(names_only_circle[0], "rb"), delimiter=",")
    entry_params = np.loadtxt(open(names_params[0], "rb"), delimiter=",")
    #entry_corrected_circle_formula = np.loadtxt(open(names_corrected_circle_formula[0], "rb"), delimiter=",")
    
    fig = plt.figure()
    ax = plt.gca()
    plt.show()
    ax.plot(entry_normal[:,0], entry_normal[:,1], Linewidth=2, Linestyle="--",color='darkorange')
    ax.plot(entry_corrected[:,0], entry_corrected[:,1], Linewidth=2, Linestyle="-",color='darkblue')
    ax.legend(["Trajectory of the robot in initial space", "Corrected trajectory with numerical estimation of the derivative"])
    
    # Plot occupied cells
    for name_cells in names_cells:
        obstacles = np.loadtxt(open(name_cells, "rb"), delimiter=",")
        for i_row in range(0,obstacles.shape[0]):
            rectangle = mpatches.Rectangle([obstacles[i_row,0]-0.5,obstacles[i_row,1]-0.5], 1, 1, color='k')
            ax.add_artist(rectangle)
                
    # Plot the borders of obstacles in range
    borders = np.loadtxt(open(names_bor[0], "rb"), delimiter=",")
    for i_row in range(0,borders.shape[0]):
        if borders[i_row,2] == 1:
            if (borders[i_row,3] == 1) and (borders[i_row,4] == 0):
                plt.plot([borders[i_row,0]-(0.5-margin), borders[i_row,0]-(0.5-margin)], [borders[i_row,1]-0.5, borders[i_row,1]+0.5], color="r", LineWidth=3)
            elif (borders[i_row,3] == -1) and (borders[i_row,4] == 0):
                plt.plot([borders[i_row,0]+(0.5-margin), borders[i_row,0]+(0.5-margin)], [borders[i_row,1]-0.5, borders[i_row,1]+0.5], color="r", LineWidth=3)
            elif (borders[i_row,3] == 0) and (borders[i_row,4] == 1):
                plt.plot([borders[i_row,0]-0.5, borders[i_row,0]+0.5], [borders[i_row,1]-(0.5-margin), borders[i_row,1]-(0.5-margin)], color="r", LineWidth=3)
            elif (borders[i_row,3] == 0) and (borders[i_row,4] == -1):
                plt.plot([borders[i_row,0]-0.5, borders[i_row,0]+0.5], [borders[i_row,1]+(0.5-margin), borders[i_row,1]+(0.5-margin)], color="r", LineWidth=3)
            else:
                print("Should not happen")
        elif borders[i_row,2] == 2:
            if borders[i_row,4] == 0:
                arc = mpatches.Arc([borders[i_row,0]-0.5, borders[i_row,1]-0.5],2*margin, 2*margin, 0, 0, 90, LineWidth=3, color="r")
            elif borders[i_row,4] == 1:
                arc = mpatches.Arc([borders[i_row,0]+0.5, borders[i_row,1]-0.5],2*margin, 2*margin, 0, 90, 180, LineWidth=3, color="r")
            elif borders[i_row,4] == 2:
                arc = mpatches.Arc([borders[i_row,0]+0.5, borders[i_row,1]+0.5],2*margin, 2*margin, 0, 180, 270, LineWidth=3, color="r")
            elif borders[i_row,4] == 3:
                arc = mpatches.Arc([borders[i_row,0]-0.5, borders[i_row,1]+0.5],2*margin, 2*margin, 0, 270, 360, LineWidth=3, color="r")
            ax.add_artist(arc)
        elif borders[i_row,2] == 3:
            if borders[i_row,4] == 0:
                arc = mpatches.Arc([borders[i_row,0]-0.5, borders[i_row,1]-0.5],2*(1-margin), 2*(1-margin), 0, 0, 90, LineWidth=3, color="r")
            elif borders[i_row,4] == 1:
                arc = mpatches.Arc([borders[i_row,0]+0.5, borders[i_row,1]-0.5],2*(1-margin), 2*(1-margin), 0, 90, 180, LineWidth=3, color="r")
            elif borders[i_row,4] == 2:
                arc = mpatches.Arc([borders[i_row,0]+0.5, borders[i_row,1]+0.5],2*(1-margin), 2*(1-margin), 0, 180, 270, LineWidth=3, color="r")
            elif borders[i_row,4] == 3:
                arc = mpatches.Arc([borders[i_row,0]-0.5, borders[i_row,1]+0.5],2*(1-margin), 2*(1-margin), 0, 270, 360, LineWidth=3, color="r")
            ax.add_artist(arc)

    fig = plt.figure()
    ax = plt.gca()
    plt.show()
    
    manager = plt.get_current_fig_manager()
    manager.window.showMaximized()


    ax.plot(entry_circle[:,0], entry_circle[:,1], Linewidth=2, Linestyle="--",color='darkorange')
    ax.plot(entry_only_circle[:,0], entry_only_circle[:,1], Linewidth=2, Linestyle="--",color='darkgreen')
    ax.plot(entry_corrected_circle[:,0], entry_corrected_circle[:,1], Linewidth=2, Linestyle="--",color='darkred')
    #ax.plot(entry_corrected_circle_formula[:,0], entry_corrected_circle_formula[:,1], Linewidth=2, Linestyle="--",color='darkblue')
    
    circle = mpatches.Ellipse([0,0], 2, 2)
    ax.add_artist(circle)
        
    # # Set axis limits and display result
    min_x = np.min([np.min(entry_circle [:,0]),np.min(entry_only_circle [:,0])])
    max_x = np.max([np.max(entry_circle [:,0]),np.max(entry_only_circle [:,0])])
    min_y = np.min([np.min(entry_circle [:,1]),np.min(entry_only_circle [:,1])])
    max_y = np.max([np.max(entry_circle [:,1]),np.max(entry_only_circle [:,1])])
    ax.set_xlim(min_x, max_x)
    ax.set_ylim(min_y, max_y)
    ax.legend(["Current trajectory of the robot in circle space", "Theoretical trajectory in circle space starting from the given initial position", "Corrected trajectory with numerical estimation of the derivative"])#, "Corrected trajectory with theoretical formula"])
    #fig.savefig("D:/Mes documents/Devoirs/MasterThesis/Article/trajectory_comparison_31_01.png", dpi=600)
    
    lim = 10000
    fig, axs = plt.subplots(3, 2)
    axs[0, 0].plot(entry_params[:lim,4], Linewidth=2, Linestyle="-",color='k') 
    axs[0, 0].set_title("Perimeter surface")
    axs[1, 0].plot(entry_params[:lim,5], Linewidth=2, Linestyle="-",color='k')
    axs[1, 0].set_title("Max gamma")
    axs[2, 0].plot(entry_params[:lim,6], Linewidth=2, Linestyle="-",color='k')
    axs[2, 0].set_title("Gamma in circle frame")
    axs[0, 1].plot(np.arccos(entry_params[:lim,0]), Linewidth=2, Linestyle="-",color='k')
    axs[0, 1].set_title("Angle theta")
    axs[1, 1].plot(np.arccos(entry_params[:lim,2]), Linewidth=2, Linestyle="-",color='k')
    axs[1, 1].set_title("Angle phi")
    axs[2, 1].plot((entry_params[:lim,7]), Linewidth=2, Linestyle="-",color='r')
    axs[2, 1].plot((entry_params[:lim,8]), Linewidth=2, Linestyle="-",color='g')
    axs[2, 1].plot((entry_params[:lim,9]), Linewidth=2, Linestyle="-",color='b')
    axs[2, 1].plot((entry_params[:lim,10]), Linewidth=2, Linestyle="-",color='k')
    axs[2, 1].legend(["A","B","C","D"])
    plt.show()
    
    fig = plt.figure()
    ax = plt.gca()
    ax.plot(entry_params[:lim,10]/entry_params[:lim,11], Linewidth=2, Linestyle="-",color='r')
    ax.plot(-entry_params[:lim,8]/entry_params[:lim,11], Linewidth=2, Linestyle="-",color='g')
    ax.plot(-entry_params[:lim,9]/entry_params[:lim,11], Linewidth=2, Linestyle="-",color='b')
    ax.plot(entry_params[:lim,7]/entry_params[:lim,11], Linewidth=2, Linestyle="-",color='k')
    ax.legend(["D/coeff","-B/coeff","-C/coeff","A/coeff"])
    plt.show()
    # # Get data and put in the the correct format
    # for name in names_normal:
    #     entry = np.loadtxt(open(name, "rb"), delimiter=",")
    #     n_size = int(len(entry[:,0])**0.5)
    #     X = np.reshape(entry[:,0], (n_size,n_size)).transpose()
    #     Y = np.reshape(entry[:,1], (n_size,n_size)).transpose()
    #     U = np.reshape(entry[:,2], (n_size,n_size)).transpose()
    #     V = np.reshape(entry[:,3], (n_size,n_size)).transpose()
    # 
    # # Create figure and get axes handle
    # fig, ax = plt.subplots()
    # 
    # # Plot occupied cells
    # for name_cells in names_cells:
    #     obstacles = np.loadtxt(open(name_cells, "rb"), delimiter=",")
    #     for i_row in range(0,obstacles.shape[0]):
    #         rectangle = mpatches.Rectangle([obstacles[i_row,0]-0.5,obstacles[i_row,1]-0.5], 1, 1, color='k')
    #         ax.add_artist(rectangle)
    #             
    # # Plot the borders of obstacles in range
    # borders = np.loadtxt(open(names_bor[0], "rb"), delimiter=",")
    # for i_row in range(0,borders.shape[0]):
    #     if borders[i_row,2] == 1:
    #         if (borders[i_row,3] == 1) and (borders[i_row,4] == 0):
    #             plt.plot([borders[i_row,0]-(0.5-margin), borders[i_row,0]-(0.5-margin)], [borders[i_row,1]-0.5, borders[i_row,1]+0.5], color="r", LineWidth=3)
    #         elif (borders[i_row,3] == -1) and (borders[i_row,4] == 0):
    #             plt.plot([borders[i_row,0]+(0.5-margin), borders[i_row,0]+(0.5-margin)], [borders[i_row,1]-0.5, borders[i_row,1]+0.5], color="r", LineWidth=3)
    #         elif (borders[i_row,3] == 0) and (borders[i_row,4] == 1):
    #             plt.plot([borders[i_row,0]-0.5, borders[i_row,0]+0.5], [borders[i_row,1]-(0.5-margin), borders[i_row,1]-(0.5-margin)], color="r", LineWidth=3)
    #         elif (borders[i_row,3] == 0) and (borders[i_row,4] == -1):
    #             plt.plot([borders[i_row,0]-0.5, borders[i_row,0]+0.5], [borders[i_row,1]+(0.5-margin), borders[i_row,1]+(0.5-margin)], color="r", LineWidth=3)
    #         else:
    #             print("Should not happen")
    #     elif borders[i_row,2] == 2:
    #         if borders[i_row,4] == 0:
    #             arc = mpatches.Arc([borders[i_row,0]-0.5, borders[i_row,1]-0.5],2*margin, 2*margin, 0, 0, 90, LineWidth=3, color="r")
    #         elif borders[i_row,4] == 1:
    #             arc = mpatches.Arc([borders[i_row,0]+0.5, borders[i_row,1]-0.5],2*margin, 2*margin, 0, 90, 180, LineWidth=3, color="r")
    #         elif borders[i_row,4] == 2:
    #             arc = mpatches.Arc([borders[i_row,0]+0.5, borders[i_row,1]+0.5],2*margin, 2*margin, 0, 180, 270, LineWidth=3, color="r")
    #         elif borders[i_row,4] == 3:
    #             arc = mpatches.Arc([borders[i_row,0]-0.5, borders[i_row,1]+0.5],2*margin, 2*margin, 0, 270, 360, LineWidth=3, color="r")
    #         ax.add_artist(arc)
    #     elif borders[i_row,2] == 3:
    #         if borders[i_row,4] == 0:
    #             arc = mpatches.Arc([borders[i_row,0]-0.5, borders[i_row,1]-0.5],2*(1-margin), 2*(1-margin), 0, 0, 90, LineWidth=3, color="r")
    #         elif borders[i_row,4] == 1:
    #             arc = mpatches.Arc([borders[i_row,0]+0.5, borders[i_row,1]-0.5],2*(1-margin), 2*(1-margin), 0, 90, 180, LineWidth=3, color="r")
    #         elif borders[i_row,4] == 2:
    #             arc = mpatches.Arc([borders[i_row,0]+0.5, borders[i_row,1]+0.5],2*(1-margin), 2*(1-margin), 0, 180, 270, LineWidth=3, color="r")
    #         elif borders[i_row,4] == 3:
    #             arc = mpatches.Arc([borders[i_row,0]-0.5, borders[i_row,1]+0.5],2*(1-margin), 2*(1-margin), 0, 270, 360, LineWidth=3, color="r")
    #         ax.add_artist(arc)
    # 
    #         
    # # Plot the stream
    # q = ax.streamplot(X, Y, U, V, density=my_density)
    # 
    # # Plot the attractor
    # ellipse = mpatches.Ellipse([attractor[num][0],attractor[num][1]], 0.2, 0.2, facecolor='forestgreen', edgecolor="k", zorder=5)
    # ax.add_artist(ellipse)
    #  
    # # Set axis limits and display result
    # min_x = np.min(entry[:,0])
    # max_x = np.max(entry[:,0])
    # min_y = np.min(entry[:,1])
    # max_y = np.max(entry[:,1])
    # 
    # ax.set_xlim(min_x, max_x)
    # ax.set_ylim(min_y, max_y)
    # #ax.set_xlim(300, 390)
    # #ax.set_ylim(280, 370)
    # ax.set_aspect("equal")
    # plt.show()
    # if save_figs:
    #     fig.savefig("/home/leziart/Pictures/Normal_VS_Bezier/normal_"+str(num)+".png", dpi=300)
    #     #fig.savefig("/home/leziart/Pictures/Normal_VS_Bezier/normal_"+str(num)+".svg", format='svg', dpi=600)
    #     fig.savefig("/home/leziart/Pictures/Normal_VS_Bezier/normal_"+str(num)+".eps", format='eps')
    
    
    
        
#disp9()
#disp6()
#disp6ter()
#disp_metrics()
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
