import math
from re import S
import numpy as np     

class Robot:
    def __init__(self,id,r_x,r_y,r_z):
        self.__id = id
        self.__x = r_x
        self.__y = r_y
        self.__z = r_z


    @property
    def id(self):
        return self.__id
    @id.setter
    def x(self,value):
        if value != "" and type(value,str):
            self.__id = value

    @property
    def x(self):
        return self.__x
    @x.setter
    def x(self,value):
        if type(value,float):
            self.__x = value
    
    @property
    def y(self):
        return self.__y
    @y.setter
    def y(self,value):
        if type(value,float):
            self.__y = value

    @property
    def z(self):
        return self.__z
    @z.setter
    def z(self,value):
        if type(value,float):
            self.__z = value

    #GET-SET METHODS
    def get_coords(self):
        return [[self.__x],[self.__y],[self.__z]]

    def set_coords(self,x,y,z):
        if not type(x,str):
            self.__x = x
        if not type(y,str):
            self.__y = y
        if not type(z,str):
            self.__z = z

    def get_distance(self,rob):
        dist = math.sqrt( (self.__x - rob.x)**2 + (self.__y - rob.y)**2 + (self.__z - rob.z)**2)
        return dist

    def print_coords(self):
        print( str(self.__id) + " coords: " + "X = " + str(self.__x) + " , Y = " + str(self.__y) + " , Z = " + str(self.__z))
        


def d_matrix(platoon):
    d_mat = np.zeros((len(platoon),len(platoon)))
    for i in range(len(platoon)):
        for j in range(len(platoon)):
            d_mat[i,j] = platoon[i].get_distance(platoon[j])
    return d_mat
    



