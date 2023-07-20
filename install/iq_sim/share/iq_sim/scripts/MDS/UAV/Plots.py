import matplotlib.pyplot as plt
import matplotlib.animation as animation
import numpy as np

def initialize_plot():
    global fig, plt

    fig = plt.figure(figsize=(5*3, 5*1))
    fig.suptitle("MDS algorithm")
    plt.ion()


def plot_uavs(true_coords, estimated_coords, waiting_time=2):

    titles = ["True position", "Estimated position", "Comparison"]

    plt.draw()
    for i in range(3):

        ax = fig.add_subplot(1,3,i+1, projection='3d')
        ax.view_init(50, -50)
        ax.set(xlabel="X", ylabel="Y", zlabel="Z", facecolor="white", title=titles[i], xlim=[-5,5], ylim=[-5,5], zlim=[-5,5])
        ax.xaxis.set_pane_color("white", alpha=None)
        ax.yaxis.set_pane_color("white", alpha=None)
        ax.zaxis.set_pane_color("white", alpha=None)

        if (i == 0): # True coordinates
            ax.scatter(true_coords[0, 1:], true_coords[1, 1:], true_coords[2, 1:], c="black")
            ax.scatter(true_coords[0, 0],  true_coords[1, 0],  true_coords[2, 0],  c="red"  )
            
        elif (i == 1): # Estimated coordinates
            ax.scatter(estimated_coords[0, 1:], estimated_coords[1, 1:], estimated_coords[2, 1:], c="blue"  )
            ax.scatter(estimated_coords[0, 0],  estimated_coords[1, 0],  estimated_coords[2, 0],  c="orange")
        else:
            # true coordinates
            ax.scatter(true_coords[0, 1:], true_coords[1, 1:], true_coords[2, 1:], c="red")
            ax.scatter(true_coords[0, 0],  true_coords[1, 0],  true_coords[2, 0],  c="black"  )
            
            # estimated coordinates
            ax.scatter(estimated_coords[0, 1:], estimated_coords[1, 1:], estimated_coords[2, 1:], c="green" )
            ax.scatter(estimated_coords[0, 0],  estimated_coords[1, 0],  estimated_coords[2, 0],  c="orange")
    
    fig.canvas.draw_idle()
    plt.pause(waiting_time)


    # scat = ax.scatter(data[0], data[1], data[2], c="b", s=5)

# line2 = ax.plot(t[0], z2[0], label=f'v0 = {v02} m/s')[0]
# ax.set(xlim=[0, 3], ylim=[-4, 10], xlabel='Time [s]', ylabel='Z [m]')
# ax.legend()


# def update(frame):
#     # for each frame, update the data stored on each artist.
#     x = data[0, :frame]
#     y = data[1, :frame]
#     z = data[2, :frame]
#     # update the scatter plot:
#     #data = np.stack([x, y]).T
#     #scat.set_offsets(data)
#     # update the line plot:
#     # scat.set_xdata(x)
#     # scat.set_ydata(y)
#     # scat.set_zdata(z)
#     ax.scatter(x,y,z)


# ani = animation.FuncAnimation(fig=fig, func=update, frames=40, interval=30)