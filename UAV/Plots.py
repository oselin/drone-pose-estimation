#!/usr/bin/env
import random


def plot_points(plt,**kwargs):
    
    if len(kwargs.items()) == 0:
         raise Exception("No data provided")
    else:
        generate_plot(plt,list(kwargs.items()))



def generate_plot(plt,data):

    # Data processing
    S = data[0][1]
    n_elem = len(S[0,:])
    n_plots = len(data)

    col = (0,0,0)  # color for the anchor: BLACK
    c_set = [col]
    
    random.seed(0)
    for _ in range(n_elem-1):
        while col in c_set:
            col = (random.random(),
                   random.random(),
                   random.random())
        c_set.append(col)


    if (n_plots==1):
        fig, axis = plt.subplots(1, figsize=(5,4))
        axis.scatter(S    [0,:], S    [1,:], color=c_set, alpha=0.2)
        axis.set_title('MDS w/ ' + title)

    else:
        
        c = 0
        fig, axis = plt.subplots(1,n_plots, figsize=(5*n_plots,4))

        for title,value in data:
            axis[c].scatter(value[0,:], value[1,:], color=c_set, alpha=1.0)
            axis[c].scatter(S    [0,:], S    [1,:], color=c_set, alpha=0.2)
            axis[c].set_title('MDS w/ ' + title)

            c += 1
    plt.show()
