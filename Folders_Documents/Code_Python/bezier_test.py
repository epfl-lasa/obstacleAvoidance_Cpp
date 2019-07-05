# clear all;close all;axis tight;hold on;
# z=5;axis(z*[-1,1,-1,1]);
# [X,Y]=getpts(); % mouse selection of points Ak ...
# n=length(X);A=[X,Y]; % ... coords placed into a n x 2 array
# u=ones(n-1,1);mat=4*eye(n)+diag(u,1)+diag(u,-1);
# mat(1,n)=1;mat(n,1)=1;mat=mat/6;P=inv(mat)*A;
# P=[P;P(1,:)];A=[A;A(1,:)];
# B=zeros(n,2);C=zeros(n,2);
# for k=1:n;
#    B(k,:)=(2*P(k,:)+P(k+1,:))/3;
#    C(k,:)=(P(k,:)+2*P(k+1,:))/3;
# end;
# step=0.01;t=(0:step:1)';s=1-t;
# s3=s.^3;s2t=3*s.^2.*t;t2s=3*t.^2.*s;t3=t.^3;
# for k=1:n
#    a=A(k,:);b=B(k,:);c=C(k,:);d=A(k+1,:);
#    bez=s3*a+s2t*b+t2s*c+t3*d;
#    plot(bez(:,1),bez(:,2),'color',.4+.4*rand(1,3),'linewidth',3)
#end


import numpy as np
from matplotlib import pyplot as plt
"""
X = np.array([0,1,1,0])
Y = np.array([0,0,1,1])
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



#plt.figure()

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
    print(bez)
    #plt.plot((bez[:,0]).transpose(),(bez[:,1]).transpose(), linewidth=3)

#plt.plot(X,Y,'o',color='k',linewidth=5)
#plt.show()

"""
##

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

import glob
import matplotlib.patches as mpatches


names = glob.glob("/home/leziart/catkin_ws/StreamNode/test_to_delete.txt")
names.sort()
data = np.loadtxt(open(names[0], "rb"), delimiter=",")

output_data = border_to_vertices(data)
output_data = np.array(output_data)

X = output_data[:,0]
Y = output_data[:,1]

n, A,B,C,s3,s2t,t2s,t3 = compute_bezier(X,Y)

fig = plt.figure()
ax = fig.gca()

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
    plt.plot((bez[:,0]).transpose(),(bez[:,1]).transpose(), linewidth=4, color='red')

for i_row in range(data.shape[0]):
    rectangle = mpatches.Rectangle([data[i_row,0]-0.5,data[i_row,1]-0.5], 1, 1, color="lightsteelblue")
    ax.add_patch(rectangle)

borders = data

for i_row in range(0,borders.shape[0]):
    if borders[i_row,2] == 1:
        if borders[i_row,3] == 0:
            plt.plot([borders[i_row,0]-0.5, borders[i_row,0]+0.5], [borders[i_row,1], borders[i_row,1]], color="k", LineWidth=2, linestyle="--")
        else:
            plt.plot([borders[i_row,0], borders[i_row,0]], [borders[i_row,1]-0.5, borders[i_row,1]+0.5], color="k", LineWidth=2, linestyle="--")
    elif borders[i_row,2] == 2:
        if borders[i_row,4] == 0:
            arc = mpatches.Arc([borders[i_row,0]-0.5, borders[i_row,1]-0.5],1, 1, 0, 0, 90, LineWidth=2, linestyle="--")
        elif borders[i_row,4] == 1:
            arc = mpatches.Arc([borders[i_row,0]+0.5, borders[i_row,1]-0.5],1, 1, 0, 90, 180, LineWidth=2, linestyle="--")
        elif borders[i_row,4] == 2:
            arc = mpatches.Arc([borders[i_row,0]+0.5, borders[i_row,1]+0.5],1, 1, 0, 180, 270, LineWidth=2, linestyle="--")
        elif borders[i_row,4] == 3:
            arc = mpatches.Arc([borders[i_row,0]-0.5, borders[i_row,1]+0.5],1, 1, 0, 270, 360, LineWidth=2, linestyle="--")
        ax.add_artist(arc)
    elif borders[i_row,2] == 3:
        if borders[i_row,4] == 0:
            arc = mpatches.Arc([borders[i_row,0]-0.5, borders[i_row,1]-0.5],1, 1, 0, 0, 90, LineWidth=2, linestyle="--")
        elif borders[i_row,4] == 1:
            arc = mpatches.Arc([borders[i_row,0]+0.5, borders[i_row,1]-0.5],1, 1, 0, 90, 180, LineWidth=2, linestyle="--")
        elif borders[i_row,4] == 2:
            arc = mpatches.Arc([borders[i_row,0]+0.5, borders[i_row,1]+0.5],1, 1, 0, 180, 270, LineWidth=2, linestyle="--")
        elif borders[i_row,4] == 3:
            arc = mpatches.Arc([borders[i_row,0]-0.5, borders[i_row,1]+0.5],1, 1, 0, 270, 360, LineWidth=2, linestyle="--")
        ax.add_artist(arc)
                                
plt.plot(X,Y,'o',color='k',linewidth=5)
#ax.set_aspect("equal")
#plt.show()

robot = np.array([[2, 5.7]])
distances = np.sqrt(np.power(robot[0,0]-output_data[:,0],2)+np.power(robot[0,1]-output_data[:,1],2))
I = np.argmin(distances)
X1 = output_data[:,0]
Y1 = output_data[:,1]
n, A,B,C,s3,s2t,t2s,t3 = compute_bezier(X1,Y1)

a=A[I-1,:]
b=B[I-1,:]
c=C[I-1,:]
d=A[I,:]
a = a.reshape((len(a),1))
b = b.reshape((len(b),1))
c = c.reshape((len(c),1))
d = d.reshape((len(d),1))
bez1= s3*a.transpose() + s2t * b.transpose() + t2s*c.transpose() + t3*d.transpose()

a=A[I,:]
b=B[I,:]
c=C[I,:]
d=A[I+1,:]
a = a.reshape((len(a),1))
b = b.reshape((len(b),1))
c = c.reshape((len(c),1))
d = d.reshape((len(d),1))
bez2= s3*a.transpose() + s2t * b.transpose() + t2s*c.transpose() + t3*d.transpose()

bez = np.vstack((bez1,bez2))
norm = np.linalg.norm(bez-robot, axis=1)
closest = bez[np.argmin(norm),:]

#fig = plt.figure()
#ax = fig.gca()
#plt.plot((bez[:,0]).transpose(),(bez[:,1]).transpose(), linewidth=3, color='red')

# Plot robot and its projection
#plt.plot(robot[0,0], robot[0,1], 'bo', linewidth=5)
#plt.plot(closest[0], closest[1], 'go', linewidth=5)
#plt.plot([robot[0,0], closest[0]],[robot[0,1],closest[1]], color='k', linestyle="--")
ax.set_aspect("equal")
plt.show()