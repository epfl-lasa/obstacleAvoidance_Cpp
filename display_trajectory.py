
import numpy as np
from matplotlib import pyplot as plt
import glob

plt.figure()

for name in glob.glob("./*.txt"):

    file = open(name, "r") 
    traj = file.readlines() 
    file.close()
    
    x = []
    y = []
    
    i = 0 
    for line in traj:
        point = line.split(',')
        x.append(float(point[0]))
        y.append(float(point[1]))
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

ax.add_artist(circle1)
ax.add_artist(circle2)
plt.axis('scaled')
plt.show()


"""import glob, os
os.chdir("/mydir")
for file in glob.glob("*.txt"):
    print(file)"""