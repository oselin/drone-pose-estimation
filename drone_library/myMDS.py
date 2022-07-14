import numpy as np                        # ndarrys for gridded data
import matplotlib.pyplot as plt           # for plotting
import math                               # trigonometry etc.
import random                             # for randon numbers

from sklearn.manifold import MDS          # multidimensional scaling
from sklearn.random_projection import GaussianRandomProjection # random projection
from sklearn.random_projection import johnson_lindenstrauss_min_dim
from sklearn.random_projection import SparseRandomProjection
from sklearn.metrics.pairwise import euclidean_distances


def main():
    random.seed(0)
    
    n = 5
    dp = np.zeros((n,n))
    for i in range(n):
        for j in range(i,n):
            if i!=j: dp[i,j] = dp[j,i] = random.random()
    print(dp)

    #MDS initialization
    embedding = MDS(n_components=2,n_init = 20,max_iter = 1000,dissimilarity='precomputed')
    MDS_transformed = embedding.fit_transform(dp)

    print(MDS_transformed.shape)



if __name__=='__main__':
    main()