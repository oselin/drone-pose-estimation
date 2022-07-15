import random
import math
import matplotlib.pyplot as plt

class Robot:
    def __init__(self,id,r_x,r_y):
        self.id = id
        self.r_x = r_x
        self.r_y = r_y
    def get_coords(self):
        print( str(self.id) + " coords: " + "X = " + str(self.r_x) + " , Y = " + str(self.r_y))
    def get_local_dist(self,platoon):
        print( str(self.id) + " distances from other robots: ")
        for i in range(n_robots):
            if self.id != i :
                print_dist(self,platoon[i])

def get_dist(r1,r2):
    dist = math.sqrt( (r1.r_x - r2.r_x)**2 + (r1.r_y - r2.r_y)**2)
    return dist
def print_dist(r1,r2):
    d = dist = math.sqrt( (r1.r_x - r2.r_x)**2 + (r1.r_y - r2.r_y)**2)
    print( str(r1.id) + " - " + str(r2.id) + " : " + str(d) )

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
for a in platoon:
    for aa in platoon:
        print_dist(a,aa)

# distances of robot 0 from all other robots
platoon[0].get_local_dist(platoon)


#plot dei punti
plt.scatter(x_coords , y_coords)
plt.show()



