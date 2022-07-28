import sys
sys.path.append('./dev_ws/src')

import numpy as np
from UAV import *
import matplotlib.pyplot as plt
import time
#GLOBAL PARAMETERS
N_ROBOTS  = 3
DIMENSION = 2


platoon = []


# INITIALIZATION OF THE ROBOTS
for i in range(N_ROBOTS):
    if i==0:
        i_robot = Robot("op_" + str(i),0,0,0)
    else:
        i_robot = Robot("op_" + str(i),np.random.uniform(0, 10.0),np.random.uniform(0, 10.0),0)
    platoon.append(i_robot)

coordinates = [[],[],[]]

for rob in platoon:
    coordinates = np.append(coordinates,rob.get_coords(),axis=1)

# Vector of true coordinates S
# NOTE: IT WILL BE USED FOR PLOTTING THE ACTUAL COORDINATES
# THE ALGORITHM NEEDS ONLY THE POSITION OF THE LEADER ANCHOR/DRONE, SO LET'S CREATE A NEW VARIABLE
# TO DEMONSTRATE IT

S = coordinates[0:2,:]
S_anc = np.copy(S)
S_anc[:,1:] = np.zeros((2,len(S[0,:])-1))

plt.ion()
ii = 1
while True:

    # Simulate the communication among UAVs and get distances
    DM = DM_from_platoon(platoon)
    
    # Add Gaussian noise: mean: 0 | variance: 1
    DM = square(DM + noise_matrix(N_ROBOTS,0,1))

    # Simulate the movement of the anchor/leader drone
    S_prime = S + move(DIMENSION,N_ROBOTS)

    # Simulate a NEW communication among UAVs and get distances
    DM_prime = square(DM_from_S(S_prime) + noise_matrix(N_ROBOTS,0,1))
    

    # Simulate a NEW movement of the anchor/leader drone to detect flip ambiguities
    S_prime2 = S_prime + move(DIMENSION,N_ROBOTS)
    # Simulate a NEW communication among UAVs and get distances
    DM_prime2 = square(DM_from_S(S_prime2) + noise_matrix(N_ROBOTS,0,1))

    SS,S_estim = MDS(S_anc,DM,S_prime,DM_prime,S_prime2,DM_prime2,DIMENSION,noise='Gaussian')
    
    plot_points(ii,plt,S=S, SS= SS,S_estim = S_estim)

    S += move(DIMENSION,N_ROBOTS,all=1)
    ii += 1
    time.sleep(4)
