from http.cookies import CookieError
from drone_library import *
import matplotlib.pyplot as plt
import random
import numpy as np


# get number of robots from user
n_robots = int(input("number of robots: "))

platoon = []

# initialization of n robots
for i in range(n_robots):
    i_robot = Robot("op_" + str(i),random.uniform(0, 10.0),random.uniform(0, 10.0))
    platoon.append(i_robot)

coordinates = np.zeros((2,1))

# create platoon of robots and the 2 arrays for all x-coords and y-coords (for the plot) 
for rob in platoon:
    coordinates = np.append(coordinates,rob.get_coords(),axis=1)
np.delete(coordinates,0)

#plot distance matrix
print(d_matrix(platoon))

#plot dei punti
plt.scatter(coordinates[0,:] , coordinates[1,:])
plt.show()

