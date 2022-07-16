import math
from re import S
import numpy as np     

class Robot:
    def __init__(self,id,r_x,r_y):
        self.id = id
        self.r_x = r_x
        self.r_y = r_y

    #GET-SET METHODS
    def get_coords(self):
        return [[self.r_x],[self.r_y]]

    def set_coords(self,x,y):
        if not type(x,str):
            self.r_x = x
        if not type(y,str):
            self.r_y = y

    def get_distance(self,rob):
        dist = math.sqrt( (self.r_x - rob.r_x)**2 + (self.r_y - rob.r_y)**2)
        return dist

    def print_coords(self):
        print( str(self.id) + " coords: " + "X = " + str(self.r_x) + " , Y = " + str(self.r_y))
        


def d_matrix(platoon):
    d_mat = np.zeros((len(platoon),len(platoon)))
    for i in range(len(platoon)):
        for j in range(len(platoon)):
            d_mat[i,j] = platoon[i].get_distance(platoon[j])
    return d_mat
    



