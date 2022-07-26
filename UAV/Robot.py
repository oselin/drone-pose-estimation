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
        if value != "" and type(value) == str:
            self.__id = value
        else:
            raise Exception("ID: bad input provided")

    @property
    def x(self):
        return self.__x
    @x.setter
    def x(self,value):
        if type(value) == float:
            self.__x = value
        elif type(value) == int:
            self.__x = float(value)
        else:
            raise Exception("X: bad input provided")
    
    @property
    def y(self):
        return self.__y
    @y.setter
    def y(self,value):
        if type(value) == float:
            self.__y = value
        elif type(value) == int:
            self.__y = float(value)
        else:
            raise Exception("Y: bad input provided")

    @property
    def z(self):
        return self.__z
    @z.setter
    def z(self,value):
        if type(value) == float:
            self.__z = value
        elif type(value) == int:
            self.__z = float(value)
        else:
            raise Exception("Z: bad input provided")
            
    #GET-SET METHODS
    def get_coords(self):
        return [[self.__x],[self.__y],[self.__z]]

    def set_coords(self,x,y,z):
        self.x = x
        self.y = y
        self.z = z

    def get_distance(self,rob):
        dist = np.sqrt( (self.__x - rob.x)**2 + (self.__y - rob.y)**2 + (self.__z - rob.z)**2)
        return dist

    def print_coords(self):
        print( str(self.__id) + " coords: " + "X = " + str(self.__x) + " , Y = " + str(self.__y) + " , Z = " + str(self.__z))