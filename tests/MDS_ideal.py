import numpy as np
from UAV import *
#GLOBAL PARAMETERS
N_ROBOTS  = 5
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
S_anc = coordinates[:,0]
print(S_anc)
print(S)

while True:

    # Simulate the communication among UAVs and get distances
    DM2 = DM_from_platoon2(platoon)


    # Simulate the movement of the anchor/leader drone
    S_prime = S + move_anchor()
    # Simulate a NEW communication among UAVs and get distances
    DM_prime = DM_from_S(S_prime)


    # Simulate a NEW movement of the anchor/leader drone to detect flip ambiguities
    S_prime2 = S_prime + move_anchor()
    # Simulate a NEW communication among UAVs and get distances
    DM_prime2 = DM_from_S(S_prime2)

    S_estim = MDS(S,DM,S_prime,DM_prime,S_prime2,DM_prime2,DIMENSION)

    plot_points(S, S_estim = S_estim)
