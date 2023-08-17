import matplotlib.pyplot as plt
import numpy as np
import matplotlib
from matplotlib import cm

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

# Initialize figure
fig = plt.figure(figsize=(6, 6))
ax = fig.add_subplot(1, 1, 1, projection='3d')

# Initialize Viridis colors
colors = cm.viridis(np.linspace(0, 1, 5))

point = [0, 0, 10]

def draw_sphere(radius, x0=0, y0=0, z0=0, color='blue', draw_line=True, draw_center=True):
    u = np.linspace(0, 2*np.pi, 100)
    v = np.linspace(0,   np.pi, 100)
    
    x = x0 + radius * np.outer(np.cos(u), np.sin(v))
    y = y0 + radius * np.outer(np.sin(u), np.sin(v))
    z = z0 + radius * np.outer(np.ones(np.size(u)), np.cos(v))

    # Draw sphere
    ax.plot_surface(x, y, z, color=color, alpha=0.05)
    
    # Draw line
    if (draw_line): ax.plot([x0, point[0]], [y0, point[1]], [z0, point[2]], color=color, marker = 'o')

    # Draw center
    if (draw_center): ax.scatter(x0, y0, z0, color=color)


# Draw first sphere
draw_sphere(radius=10.04987562112089, x0=1, y0=0, z0=0, color=colors[0])

# Draw second sphere
draw_sphere(radius=9.433981132056603, x0=6, y0=2, z0=3, color=colors[1])

# Draw third sphere
draw_sphere(radius=6.708203932499369, x0=2, y0=4, z0=5, color=colors[2])

# Draw fourth sphere
draw_sphere(radius=13.341664064126334, x0=4, y0=9, z0=1, color=colors[3])

# Draw detected point
ax.scatter(point[0], point[1], point[2], color='red')

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
ax.set_title("Trilateration example")

ax.view_init(22, -20)

plt.savefig(PATH + 'trilateration.pdf', dpi=300, format='pdf', bbox_inches='tight')
plt.show()