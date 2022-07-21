import matplotlib.pyplot as plt
import random


def plot_points(S, **kwargs):
    
    col = (0,0,0)
    c_set = [col]

    for _ in range(len(S[1,:])-1):
        while col in c_set:
            col = (random.random(),
                    random.random(),
                    random.random())
        c_set.append(col)

    c = 0
    
    nargs = len(kwargs.items())

    if (nargs == 0): 
        fig = plt.figure(1,(5,4))
        ax = fig.add_subplot(111)
        
        plt.scatter(S    [0,:], S    [1,:], color=c_set, alpha=1.0)

        plt.title('Original points (theoretically unknown)')
        fig.subplots_adjust(wspace=.4, hspace=0.5)
        plt.show()

        return

    
    fig,axis = plt.subplots(1,nargs, figsize=(5*nargs,4))

    for title,value in kwargs.items():

        axis[c].scatter(value[0,:], value[1,:], color=c_set, alpha=1.0)
        axis[c].scatter(S    [0,:], S    [1,:], color=c_set, alpha=0.2)
        axis[c].set_title('MDS w/ ' + title)

        c += 1
    plt.show()
