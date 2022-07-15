import numpy as np                        # ndarrys for gridded data
import matplotlib.pyplot as plt           # for plotting
import math                               # trigonometry etc.
import random                             # for randon numbers
import seaborn as sns                     # for matrix scatter plots

from sklearn.manifold import MDS          # multidimensional scaling
from sklearn.random_projection import GaussianRandomProjection # random projection
from sklearn.random_projection import johnson_lindenstrauss_min_dim
from sklearn.random_projection import SparseRandomProjection
from sklearn.metrics.pairwise import euclidean_distances


def main():
    random.seed(0)
    dpalette = sns.color_palette("rocket_r",n_colors = 4)
    
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

    plt.subplot(121)
    pairplot = sns.scatterplot(x = MDS_transformed[:,0],y = MDS_transformed[:,1],markers='o',palette = dpalette,edgecolor="black")

    plt.subplot(122)
    pairplot = sns.scatterplot(x = MDS_transformed[:,0],y = MDS_transformed[:,1],markers='o',edgecolor="black")

    plt.subplots_adjust(left=0.0, bottom=0.0, right=2., top=1.3, wspace=0.3, hspace=0.2,)
    plt.show()



if __name__=='__main__':
    main()