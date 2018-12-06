import matplotlib.pyplot as plt
import matplotlib
from mpl_toolkits.mplot3d import Axes3D
import numpy as np

data = np.genfromtxt('balloon_trajectory.csv',delimiter=',')
data[:,0] -= np.min(data[:,0])
fig = plt.figure()
ax = fig.add_subplot(111,projection='3d')

p = ax.scatter(data[:,1],data[:,2],data[:,3],c=data[:,0],cmap=matplotlib.cm.inferno)
ax.auto_scale_xyz([-3,3],[-3,3],[0,3])
plt.colorbar(p)
plt.show()

