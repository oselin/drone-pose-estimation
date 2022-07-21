import matplotlib.pyplot as plt
import UAV.Algebra as alg




def plot_points(S, **kwargs):
    
    match = c = 0

    if len(S[0,:]) == 3:
        c_set = ['black','blue','orange']
    else:
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

  

    if list(kwargs.items())[-1][0] == 'match':
        
        if type(list(kwargs.items())[-1][1]) == int:
            match = list(kwargs.items())[-1][1]
        nargs -= 1
    
    
    fig,axis = plt.subplots((match+1),nargs, figsize=(5*nargs,4*(match+1)))

    for title,value in kwargs.items():

        if c >= nargs: break
        
        if match:
            axis[0,c].scatter(value[0,:], value[1,:], color=c_set, alpha=1.0)
            axis[0,c].scatter(S    [0,:], S    [1,:], color=c_set, alpha=0.2)
            axis[0,c].set_title('MDS w/ ' + title)

            axis[1,c].scatter(alg.match_anchor(S,value)[0,:], alg.match_anchor(S,value)[1,:], color=c_set, alpha=1.0)
            axis[1,c].scatter(S    [0,:], S    [1,:], color=c_set, alpha=0.2)
            axis[1,c].set_title('Overlapped anchor, MDS w/ ' + title)
        
        else:
            axis[c].scatter(value[0,:], value[1,:], color=c_set, alpha=1.0)
            axis[c].scatter(S    [0,:], S    [1,:], color=c_set, alpha=0.2)
            axis[c].set_title('MDS w/ ' + title)
        c += 1

    plt.show()
