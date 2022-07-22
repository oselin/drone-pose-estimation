import numpy as np
from UAV import *
import random

#GLOBAL PARAMETERS
N_ROBOTS  = 5
DIMENSION = 2


platoon = []


# INITIALIZATION OF THE ROBOTS
for i in range(N_ROBOTS):
    if i==0:
        i_robot = Robot("op_" + str(i),0,0,0)
    else:
        i_robot = Robot("op_" + str(i),random.uniform(0, 10.0),random.uniform(0, 10.0),0)
    platoon.append(i_robot)

coordinates = [[],[],[]]

for rob in platoon:
    coordinates = np.append(coordinates,rob.get_coords(),axis=1)



# Vector of true coordinates S
S = coordinates[0:2,:]

# Definition of the distance amtrix
DM2 = d_matrix2(platoon)

# MDS algorithm: vector of estimated coordinate S*
S_star = EVD(DM2,DIMENSION)

# Remove translational ambiguities
S_star = match_anchor(S,S_star,verbose=1)

# First displacement
deltaX1 = 0.3
deltaY1 = 0.1

DeltaS_prime = np.array([[0 for i in range(N_ROBOTS)] for j in range(2)],dtype=float)
DeltaS_prime[:,0] = [deltaX1,deltaY1]

# New coordinates: S'
S_prime = S + DeltaS_prime

# New distance matrix: DM'
DM_prime = DM_from_S(S_prime,verbose=1)

# Estimation of the rotation angle: theta_r
theta_r   = estimate_theta(DM2,DM_prime,S_star,[deltaX1,deltaY1],verbose=1)
theta = LSE(DM2,DM_prime,S_star,[deltaX1,deltaY1])

# New coordinates: S**
S_star2 = rotateMatrix(theta_r)@S_star
S_starMin = rotateMatrix(theta.x)@S_star

# Understand if there exist flip ambiguities
deltaX2 = 0.7
deltaY2 = 0.4

DeltaS_prime2 = np.array([[0 for i in range(N_ROBOTS)] for j in range(2)],dtype=float)
DeltaS_prime2[:,0] = [deltaX2,deltaY2]

S_prime2 = S + DeltaS_prime2

DM_prime2 = DM_from_S(S_prime2,verbose=1)

theta_r2 = estimate_theta(DM2,DM_prime2,S_star2,[deltaX2,deltaY2],approx=2,verbose=0)
print("theta_r2 : ", theta_r2)


if (theta_r2 != 0):
    F = np.array([[-1,0],[0,1]])
    
    theta_r3 = estimate_theta(DM2,DM_prime,F@S_star,[deltaX1,deltaY1],verbose=0)
    theta = LSE(DM2,DM_prime,F@S_star,[deltaX1,deltaY1])

    print("Wrong estimated angle:",theta_r)
    print("True  estimated angle:", theta_r3)

    # MDS algorithm: vector of estimated coordinate S*
    S_star = F@EVD(DM2,DIMENSION)

    # Remove translational ambiguities
    S_star = match_anchor(S,S_star,verbose=1)

    S_star2 = rotateMatrix(theta_r3)@S_star
    S_starMin = rotateMatrix(theta.x)@S_star


plot_points(S,S_star  = S_star,
              S_star2 = S_star2,
              S_starMin = S_starMin
            )