import matplotlib.pyplot as plt    




def plot_points(S, **kwargs):
    
    c_set = ['black'] + ['green' for i in range(1,len(S[0,:]))]
    
    nargs = len(kwargs.items())

    if (nargs == 0): 
        fig = plt.figure(1,(5,4))
        ax = fig.add_subplot(111)
        
        plt.scatter(S    [0,:], S    [1,:], color=c_set, alpha=1.0)

        plt.title('Original points (theoretically unknown)')
        fig.subplots_adjust(wspace=.4, hspace=0.5)
        plt.show()

        return

    fig = plt.figure(nargs, (5*nargs,4))

    c = 0

    for title,value in kwargs.items():

        c += 1
        plt_index = int('%i%i%i' % (1,nargs,c))
        ax = fig.add_subplot(plt_index)
        
        plt.scatter(value[0,:], value[1,:], color=c_set, alpha=1.0)
        plt.scatter(S    [0,:], S    [1,:], color=c_set, alpha=0.2)

        plt.title('MDS w/ ' + title)
        fig.subplots_adjust(wspace=.4, hspace=0.5)

    plt.show()
