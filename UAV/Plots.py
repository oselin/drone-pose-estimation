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
    
    # Ugly approach to always set the same colors
    random.seed(0)
    for _ in range(n_elem-1):
        while col in c_set:
            col = (random.random(),
                   random.random(),
                   random.random())
        c_set.append(col)

    fig, axis = plt.subplots(1,n_plots, figsize=(5*n_plots,4),num=1, clear=True)

    c = 0
    xl = [min(S[0,:])-2,max(S[0,:])+2]
    yl = [min(S[1,:])-2,max(S[1,:])+2]
    for title,value in data:
        axis[c].scatter(value[0,:], value[1,:], color=c_set, alpha=1.0)
        axis[c].scatter(S    [0,:], S    [1,:], color=c_set, alpha=0.2)
        axis[c].set_title('MDS w/ ' + title)
        axis[c].set_xlim(xl)
        axis[c].set_ylim(yl)

        c += 1
    plt.show()

    # Define and update plot
    plt.pause(0.001)
