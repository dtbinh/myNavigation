import numpy as np
import matplotlib.pyplot as plt

def plot_trajectory(position, waypoints=None):
    x, y = zip(*position)
    plt.plot(x,y,'.');
    if not waypoints is None:
        x, y = zip(*waypoints)
        plt.plot(x,y,'*')
    plt.axis('equal');
    plt.show()

def plot_error(waypoints, ref):
    error = np.linalg.norm(np.array(waypoints) - np.array(ref), axis=1)
    plt.plot(error,'-*',mec='orange',mfc='orange');
    plt.show()
    
def plot_cleaning(trajectory):
    x, y = zip(*trajectory)
    r = 0.354/2
    n = len(x)
    plt.axis([-5,5,-13,2]);
    plt.axis('square');
    ax = plt.gca()
    ax.cla()
    ax.set_xlim((-5, 5))
    ax.set_ylim((-13, 2))
    for i in range(0,n):
        ax.add_artist(plt.Circle((x[i],y[i]),r))
    plt.show()
        