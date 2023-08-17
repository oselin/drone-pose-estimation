import matplotlib.pyplot as plt
import numpy as np
import matplotlib
from matplotlib import cm
from scipy.stats import qmc
import os

try:
    matplotlib.use('TkAgg')
except:
    raise SystemError(
        "Impossible to start class with TkAgg as X-server")


# Latex settings
params = {"ytick.color" : "black",
          "xtick.color" : "black",
          "axes.labelcolor" : "black",
          "axes.edgecolor" : "black",
          "text.usetex" : True,
          "font.family" : "serif",
          "font.serif" : ["Computer Modern Serif"]}
plt.rcParams.update(params)


PATH = 'add/your/custom/path/'
N_POINTS = 10

# Latin Hypercube Sampling
def latin_hypercube_sampling(sampling_budget:int , dimension=2, boundaries=[[-5,-5],[5,5]]):
    """
    Generate a Latin Hypercube Sample of size n and dimension d.

    Parameters:
    - n_samples (int): The number of samples to generate.
    - dimension (int): The dimension of each sample.
    - lower_bounds: List[float] N-dimensional array with the lower bounds values
    - upper_bounds: List[float] N-dimensional array with the upper bounds values

    Returns:
    - numpy.ndarray: A n-by-d matrix of samples, where each row is a sample of length d.
    """

    boundaries = np.array(boundaries)

    samples = qmc.LatinHypercube(scramble=False,d=dimension)
    samples = samples.random(sampling_budget)
    samples = qmc.scale(samples, boundaries[0], boundaries[1])

    return samples


# Initialize figure
fig = plt.figure(figsize=(6, 6))
ax = fig.add_subplot(1, 1, 1, projection='3d')

# Initialize Viridis colors
colors = cm.viridis(np.linspace(0, 1, N_POINTS))

points = latin_hypercube_sampling(N_POINTS, dimension=3, boundaries=[[-10,-10, -10], [10, 10, 10]]).T

for i in range(N_POINTS):

    p = points[:, i]

    for j in range(N_POINTS):
        if (i != j): # avoid plotting a point connected to itself
            q = points[:,j]
            ax.plot([p[0],q[0]],[p[1],q[1]],[p[2],q[2]], color=colors[i], marker = 'o')


# Background color
ax.xaxis.set_pane_color("white", alpha=None)
ax.yaxis.set_pane_color("white", alpha=None)
ax.zaxis.set_pane_color("white", alpha=None)

# Graph limits
ax.set_xlim([-10,10])
ax.set_ylim([-10,10])
ax.set_zlim([-10,10])

ax.set_aspect('equal', 'box')

# Labels and title
ax.set_xlabel('X')
ax.set_ylabel('Y')
ax.set_zlabel('Z')
ax.set_title("Example of fully-connected points")

ax.view_init(22, -20)

plt.savefig(PATH + 'MDS_visualization.pdf', dpi=300, format='pdf', bbox_inches='tight')
plt.show()