import sys, time, ast
sys.path.append('')

import numpy as np
from UAV import *
import matplotlib.pyplot as plt

np.random.seed(10)

def simulate(parameters):
    
    # Initialize UAVs coordinates, randomly
    X = np.random.uniform(low = -5, high=5, size=[3, parameters['number_uavs']])

    alpha = 1
    initialize_plot()

    #
    # ANCHOR - position of the anchor, after the applied motion
    # X      - coordinates of the fleet
    #

    # Retrieve the distances and build the distance matrix DM. In reality it comes from UWB sensors
    ANCHOR1, X = move_anchor(points = X, step = 0)
    DM1  = distance_matrix(X)     

    # Simulate a second virtual anchor, by moving the real one and retrieving distances
    ANCHOR2, X  = move_anchor(points = X, step = 1, displacement=alpha)
    DM2 = distance_matrix(X)      
    
    # Simulate a third virtual anchor, by moving the real one and retrieving distances
    ANCHOR3, X  = move_anchor(points = X, step = 2, displacement=alpha)
    DM3 = distance_matrix(X)    
    
    # Simulate a fourth virtual anchor, by moving the real one and retrieving distances
    ANCHOR4, X  = move_anchor(points = X, step = 3, displacement=alpha)
    DM4 = distance_matrix(X)
    
    # Assemble the distance information in one unique matrix
    DM = combine_matrices(DM1, DM2, DM3, DM4, ANCHOR1, ANCHOR2, ANCHOR3, ANCHOR4)

    # Store the anchor and virtual anchors position into a coordinates array
    anchor_pos = np.hstack([ANCHOR1, ANCHOR2, ANCHOR3, ANCHOR4])

    # Return to the initial point
    _, X = move_anchor(points = X, step = 4, displacement=alpha)

    # Estimate the fleet coordinates
    X_hat_MDS = MDS(DM, anchor_pos)
    X_hat_WLP = WLP(DM, anchor_pos)

    return X, X_hat_MDS, X_hat_WLP
    
    # Plot the scenario
    # plot_uavs(true_coords=X, estimated_coords=X_hat)
    
    # Make the fleet move, except for the anchor
    # X = move_fleet(points = X, low = -2, high = 2)


def simulate_noise(parameters):
    
    # Initialize UAVs coordinates, randomly
    X = np.random.uniform(low = -5, high=5, size=[3, parameters['number_uavs']])

    alpha = 1
    mean, sigma = 0, 0.05
    initialize_plot()

    #
    # ANCHOR - position of the anchor, after the applied motion
    # X      - coordinates of the fleet
    #

    # Retrieve the distances and build the distance matrix DM. In reality it comes from UWB sensors
    ANCHOR1, X = move_anchor(points = X, step = 0)
    DM1  = distance_matrix(X) + noise(mean=mean, std=sigma, shape=parameters['number_uavs'])       

    # Simulate a second virtual anchor, by moving the real one and retrieving distances
    ANCHOR2, X  = move_anchor(points = X, step = 1, displacement=alpha)
    DM2 = distance_matrix(X) + noise(mean=mean, std=sigma, shape=parameters['number_uavs'])          
    
    # Simulate a third virtual anchor, by moving the real one and retrieving distances
    ANCHOR3, X  = move_anchor(points = X, step = 2, displacement=alpha)
    DM3 = distance_matrix(X) + noise(mean=mean, std=sigma, shape=parameters['number_uavs'])          
    
    # Simulate a fourth virtual anchor, by moving the real one and retrieving distances
    ANCHOR4, X  = move_anchor(points = X, step = 3, displacement=alpha)
    DM4 = distance_matrix(X)+ noise(mean=mean, std=sigma, shape=parameters['number_uavs'])       
    
    # Assemble the distance information in one unique matrix
    DM = combine_matrices(DM1, DM2, DM3, DM4, ANCHOR1, ANCHOR2, ANCHOR3, ANCHOR4)

    # Store the anchor and virtual anchors position into a coordinates array
    anchor_pos = np.hstack([ANCHOR1, ANCHOR2, ANCHOR3, ANCHOR4])

    # Return to the initial point
    _, X = move_anchor(points = X, step = 4, displacement=alpha)

    # Estimate the fleet coordinates
    X_hat_MDS = MDS(DM, anchor_pos)
    X_hat_WLP = WLP(DM, anchor_pos)

    return X, X_hat_MDS, X_hat_WLP