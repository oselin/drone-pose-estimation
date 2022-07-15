from drone_library import *
import matplotlib.pyplot as plt
import random

from drone_library.Robot import d_matrix

# get number of robots from user
n_robots = int(input("number of robots: "))

platoon = []

# initialization of n robots
for i in range(n_robots):
    i_robot = Robot("op_" + str(i),random.uniform(0, 10.0),random.uniform(0, 10.0))
    platoon.append(i_robot)

x_coords = []
y_coords = []

# create platoon of robots and the 2 arrays for all x-coords and y-coords (for the plot) 
for rob in platoon:
    x_coords.append(rob.r_x)
    y_coords.append(rob.r_y)
    rob.get_coords()

# get the distances between all robots
#for a in platoon:
#    for aa in platoon:
#        print_dist(a,aa)

# distances of robot 0 from all other robots
#platoon[0].get_local_dist(platoon)

#plot distance matrix
d_matrix(platoon)

#plot dei punti
plt.scatter(x_coords , y_coords)
plt.show()

