import math

class Robot:
    def __init__(self,id,r_x,r_y):
        self.id = id
        self.r_x = r_x
        self.r_y = r_y

    def get_coords(self):
        print( str(self.id) + " coords: " + "X = " + str(self.r_x) + " , Y = " + str(self.r_y))
        
    def get_local_dist(self,platoon):
        print( str(self.id) + " distances from other robots: ")
        for i in range(len(platoon)):
            if self.id != i :
                print_dist(self,platoon[i])

def get_dist(r1,r2):
    dist = math.sqrt( (r1.r_x - r2.r_x)**2 + (r1.r_y - r2.r_y)**2)
    return dist
    
    
def print_dist(r1,r2):
    d = dist = math.sqrt( (r1.r_x - r2.r_x)**2 + (r1.r_y - r2.r_y)**2)
    print( str(r1.id) + " - " + str(r2.id) + " : " + str(d) )




