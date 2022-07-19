import numpy as np

def d_matrix(platoon):
    d_mat = np.zeros((len(platoon),len(platoon)))
    for i in range(len(platoon)):
        for j in range(len(platoon)):
            d_mat[i,j] = platoon[i].get_distance(platoon[j])
    return d_mat
    

def d_matrix2(platoon):
    d_mat = np.zeros((len(platoon),len(platoon)))
    for i in range(len(platoon)):
        for j in range(len(platoon)):
            d_mat[i,j] = platoon[i].get_distance(platoon[j])**2
    return d_mat


def rotateMatrix(theta):
    return np.array([[np.cos(theta),np.sin(theta)],[-np.sin(theta),np.cos(theta)]])


def g(t):
    return t - 2*np.pi*np.floor(t/(2*np.pi)+1/2)


def theta_i1(Sstar,S,index):
    if index > 0:
        return g(np.arctan2(Sstar[1,index],Sstar[0,index])-np.arctan2(S[1,index],S[0,index]))
    else:
        return None


def THETA_i(S,displ,index):
    deltaX = displ[0]
    deltaY = displ[1]
    return np.arctan2(S[1,index],S[0,index]) + np.arctan2(deltaX,deltaY) - np.pi/2


def theta_i2(theta1, THETA):
    return g(theta1 + 2*THETA) 