import numpy as np                        # ndarrys for gridded data
from mpl_toolkits import mplot3d
import matplotlib.pyplot as plt           # for plotting
import math                               # trigonometry etc.
import random                             # for randon numbers
import seaborn as sns                     # for matrix scatter plots

from sklearn.manifold import MDS          # multidimensional scaling
from sklearn.random_projection import GaussianRandomProjection # random projection
from sklearn.random_projection import johnson_lindenstrauss_min_dim
from sklearn.random_projection import SparseRandomProjection
from sklearn.metrics.pairwise import euclidean_distances

from UAV import *


def main():
    random.seed(0)
    dpalette = sns.color_palette("rocket_r",n_colors = 4)
    
    # get number of robots from user
    n_robots = int(input("number of robots: "))

    platoon = []
    coordinates = [[],[],[]]

    # initialization of n robots
    for i in range(n_robots):
        i_robot = Robot("op_" + str(i),random.uniform(0, 10.0),random.uniform(0, 10.0),0)
        platoon.append(i_robot)

    # create platoon of robots and the 2 arrays for all x-coords and y-coords (for the plot) 
    for rob in platoon:
        coordinates = np.append(coordinates,rob.get_coords(),axis=1)

    print(coordinates)
    #Create the distance matrix
    dm = d_matrix(platoon)
   
    #MDS initialization
    embedding = MDS(n_components=2,n_init = 20,max_iter = 1000,dissimilarity='precomputed')
    MDS_transformed = embedding.fit_transform(dm)
    print(MDS_transformed.shape)

    fig = plt.figure(2, (10,4))
    ax = fig.add_subplot(121, projection='3d')
    ax.scatter3D(coordinates[0,:], coordinates[1,:], coordinates[2,:])
    plt.title('Original Points')

    ax = fig.add_subplot(122)
    plt.scatter(MDS_transformed[:,0], MDS_transformed[:,1])
    plt.scatter(coordinates[0,:], coordinates[1,:], color='red')
    plt.title('Embedding in 2D')
    fig.subplots_adjust(wspace=.4, hspace=0.5)
    plt.show()



if __name__=='__main__':
    main()
