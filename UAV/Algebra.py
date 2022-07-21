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
    print(index)
    if index > 0:
        return g(np.arctan2(Sstar[1,index],Sstar[0,index])-np.arctan2(S[1,index],S[0,index]))
    else:
        print("OK")
        return None


def THETA_i(S,displ,index):
    deltaX = displ[0]
    deltaY = displ[1]
    return np.arctan2(S[1,index],S[0,index]) + np.arctan2(deltaY,deltaX) - np.pi/2


def theta_i2(theta1, THETA):
    return g(theta1 + 2*THETA) 


def estimate_theta(DM2,DMprime,Sstar,displ,index=1,approx = 0,verbose=0):

    deltaX = displ[0]
    deltaY = displ[1]

    a2 = DM2[0,index] - DMprime[0,index] + deltaX**2 + deltaY**2
    b2 = -2*(Sstar[0,index]*deltaX + Sstar[1,index]*deltaY)         
    c2 =  2*(Sstar[0,index]*deltaY - Sstar[1,index]*deltaX)     

    a3 = DM2[0,index+1] - DMprime[0,index+1] + deltaX**2 + deltaY**2
    b3 = -2*(Sstar[0,index+1]*deltaX + Sstar[1,index+1]*deltaY)     
    c3 =  2*(Sstar[0,index+1]*deltaY - Sstar[1,index+1]*deltaX)     

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
    if approx == 1:
        atheta = round(theta,4)
    elif approx == 2:
        atheta = round(theta,1)
    if verbose > 0:
        print("After normalization: ")
        print("cos(theta):\t%f" % (cosTheta))
        print("sin(theta):\t%f" % (sinTheta))
        print("\nActual theta:\t",theta)
        
        if approx > 0:
            print("Approx theta:\t",atheta)
        else:
            print("Approx theta:\tDisabled")

    if approx > 0:
        return atheta
    else:
        return theta

def estimate_theta2(DM2,DMprime,Sstar,displ,index=1,verbose=0):
    
    deltaX = displ[0]
    deltaY = displ[1]

    a = DM2[0,index] - DMprime[0,index] + deltaX**2 + deltaY**2
    b = -2*(Sstar[0,index]*deltaX + Sstar[1,index]*deltaY)    
    c =  2*(Sstar[0,index]*deltaY - Sstar[1,index]*deltaX)    

    theta1 = np.arctan((-a*c**2-np.sqrt(-c**2*(a**2-b**2-c**2))*b)/(b**2+c**2)/c,(-a*b+np.sqrt(-c**2*(a**2-b**2-c**2)))/(b**2+c**2))
    theta2 = np.arctan((-a*c**2+np.sqrt(-c**2*(a**2-b**2-c**2))*b)/(b**2+c**2)/c,(-a*b-np.sqrt(-c**2*(a**2-b**2-c**2)))/(b**2+c**2))

    return theta1,theta2

def estimate_theta3(DM2,DMprime,Sstar,displ,index=1,verbose=0):
    
    deltaX = displ[0]
    deltaY = displ[1]

    a = DM2[0,index] - DMprime[0,index] + deltaX**2 + deltaY**2
    b = -2*(Sstar[0,index]*deltaX + Sstar[1,index]*deltaY)    
    c =  2*(Sstar[0,index]*deltaY - Sstar[1,index]*deltaX)    

    theta1 = np.arctan2(a*b + np.abs(c)*np.sqrt(b**2+c**2-a**2),a*c - b/c*np.abs(c)*np.sqrt(b**2+c**2-a**2))
    theta2 = np.arctan2(a*b - np.abs(c)*np.sqrt(b**2+c**2-a**2),a*c + b/c*np.abs(c)*np.sqrt(b**2+c**2-a**2))

    return theta1,theta2

def DM_from_S(S,verbose=0):
    e   = np.array([[1] for i in S[0,:]])

    Phiprime = np.array([np.diag(S.T@S)]).T
    
    DMprime = Phiprime@e.T - 2*S.T@S + e@Phiprime.T

    if verbose > 1:
        print("Phi':\n",Phiprime)
    
    if verbose > 0:
        print("DM' :\n",DMprime)
    
    return DMprime



def EVD(DM,final_dimension):

    n = len(DM) #since it is a square matrix, no need to specify len for columns or rows
    
    # Centering Matrix definition
    H = np.identity(n) - np.ones((n,n))

    # Double centered matrix
    B = -1/2*H@DM@H

    # Eiegenvalue decomposition
    ev, EV = np.linalg.eig(B)

    LAMBDA = np.eye(final_dimension)
    U = np.zeros((n,final_dimension))

    for i in range(final_dimension):
        # Search for the heighest eigenvalue. Put it into lambda and its associated vector in U.
        # Eventually set the eigenvalue to -1000 to find the other heighest eigenvalues
        ind  = np.argmax(ev)
        LAMBDA[i,i] = ev[ind]
        U[:,i] = EV[:,ind]
        ev[ind] = -1000


    S_star = np.sqrt(LAMBDA)@U.T

    return S_star


def match_anchor(S,S_star, verbose = 0):

    displX = S[0,0] - S_star[0,0]
    displY = S[1,0] - S_star[1,0]

    displacement_matrix = np.array([[displX for _ in range(len(S[0,:]))],
                                    [displY for _ in range(len(S[0,:]))]])

    if (verbose == 1):
        print("X displacement: ", displX)
        print("Y displacement: ", displY)

        print("Displacement matrix:\n",displacement_matrix)
    return S_star + displacement_matrix