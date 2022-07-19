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
    if (t >= -np.pi and t < np.pi):
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


def estimate_theta(DM2,DMprime,Sstar,displ,index=1,verbose=0):

    deltaX = displ[0]
    deltaY = displ[1]

    a2 = DM2[0,index] - DMprime[0,index] + deltaX**2 + deltaY**2
    b2 = -2*(Sstar[0,index]*deltaX + Sstar[1,index]*deltaY)    
    c2 =  2*(Sstar[0,index]*deltaY - Sstar[1,index]*deltaX)    

    a3 = DM2[0,index + 1] - DMprime[0,index + 1] + deltaX**2 + deltaY**2
    b3 = -2*(Sstar[0,index + 1]*deltaX + Sstar[1,index + 1]*deltaY)     
    c3 =  2*(Sstar[0,index + 1]*deltaY - Sstar[1,index + 1]*deltaX)    

    sinTheta = (a3*b2-a2*b3)/(b3*c2-b2*c3)
    cosTheta = (a2*c3-a3*c2)/(b3*c2-b2*c3)

    if verbose > 0:
        print("Coordinates to work with: %i-th and %i-th" % (index, index+1))

    if verbose > 1:
        print("Before normalization: ")
        print("cos(theta):\t%f" % (cosTheta))
        print("sin(theta):\t%f" % (sinTheta))

    #NORMALIZATION
    mod = np.sqrt(sinTheta**2+cosTheta**2)
    sinTheta /= mod
    cosTheta /= mod

    theta = np.arctan2(sinTheta,cosTheta)
    atheta = round(theta,4)
    if verbose > 0:
        print("After normalization: ")
        print("cos(theta):\t%f" % (cosTheta))
        print("sin(theta):\t%f" % (sinTheta))
        print("\nActual theta:\t",theta)
        print("Approx theta:\t",atheta)

    return atheta

def estimate_theta2(DM2,DMprime,Sstar,displ,index=1,verbose=0):
    
    deltaX = displ[0]
    deltaY = displ[1]

    a = DM2[0,index] - DMprime[0,index] + deltaX**2 + deltaY**2
    b = -2*(Sstar[0,index]*deltaX + Sstar[1,index]*deltaY)    
    c =  2*(Sstar[0,index]*deltaY - Sstar[1,index]*deltaX)    

    theta1 = np.arctan2((-a*c**2-np.sqrt(-c**2*(a**2-b**2-c**2))*b)/(b**2+c**2)/c,(-a*b+np.sqrt(-c**2*(a**2-b**2-c**2)))/(b**2+c**2))
    theta2 = np.arctan2((-a*c**2+np.sqrt(-c**2*(a**2-b**2-c**2))*b)/(b**2+c**2)/c,(-a*b-np.sqrt(-c**2*(a**2-b**2-c**2)))/(b**2+c**2))

    return [theta1,theta2]

def DM_from_S(S,verbose=0):
    e   = np.array([[1] for i in S[0,:]])

    Phiprime = np.array([np.diag(S.T@S)]).T
    
    DMprime = Phiprime@e.T - 2*S.T@S + e@Phiprime.T

    if verbose > 1:
        print("Phi':\n",Phiprime)
    
    if verbose > 0:
        print("DM' :\n",DMprime)
    
    return DMprime