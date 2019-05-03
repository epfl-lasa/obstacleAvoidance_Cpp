## P-A Leziart, 24 April 2019
# Goal: Check the accuracy of the realsense camera depending on the position
# of the area of interest in the field of view

from matplotlib import pyplot as plt
import numpy as np
import glob


# Detect text files
names = glob.glob("/home/leziart/Documents/AnalysisAccuracyRealsense/Data_Accuracy/data*.txt")
names.sort()

if True:
    # Extract data from text files
    storage = np.zeros((len(names),307200))
    for i in range(len(names)):
        temp = np.loadtxt(open(names[i], "rb"), delimiter=",")
        storage[i,:] = temp[:,4]
else:
    # Load "storage_wall.npy"
    storage = np.load("storage_wall.npy")
  
deviation = np.nanstd(storage, axis=0)

temp = np.loadtxt(open(names[0], "rb"), delimiter=",")

data = np.zeros((3,307200))
data[0,:] = temp[:,0]
data[1,:] = temp[:,1]
data[2,:] = deviation

colors= data[2,:]
colors= np.minimum(data[2,:],1.25*np.ones(307200))

fig = plt.figure()
ax = fig.gca()
c = ax.scatter(data[0,:], np.flip((data[1,:])), c=colors, cmap='gnuplot') 
cbar = fig.colorbar(c)
plt.xlabel("Position along X [pixels]")
plt.ylabel("Position along Y [pixels]")
plt.title("Standard deviation of the depth depending on the position in the field of view [m]")
plt.show()


scene = np.loadtxt(open(names[0], "rb"), delimiter=",")
scene = scene[:,4].transpose()
#data[np.isnan(data)] = 0
scene = np.reshape(scene,(640,480))
scene = scene.transpose()
fig = plt.figure()
ax = fig.gca()
plt.imshow(scene)
plt.show()