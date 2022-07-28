#!/usr/bin/env
import numpy as np
from  scipy.optimize import minimize_scalar


def EVD(DM,final_dimension):

    n = len(DM) #since it is a square matrix, no need to specify len for columns or rows

    # Centering Matrix definition
    H = np.identity(n) - np.ones((n,n))/n

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
        LAMBDA[i,i] = ev[ind].real
        U[:,[i]]  = EV[:,[ind]].real
        ev[ind] = -1000
    
    S_star = np.sqrt(LAMBDA)@U.T
    return S_star


def remove_offset(S,S_star, verbose = 0):

    # Find the offset between the 2 anchors
    displX = S[0,0] - S_star[0,0]
    displY = S[1,0] - S_star[1,0]

    # Generate a displacement matrix
    displacement_matrix = np.array([[displX for _ in range(len(S[0,:]))],
                                    [displY for _ in range(len(S[0,:]))]])

    if (verbose == 1):
        print("X displacement: ", displX)
        print("Y displacement: ", displY)
        print("\nDisplacement matrix:\n",displacement_matrix)

        print('Sx : ',S[0,0])
        print('S*x: ',S_star[0,0])
        print('Sy :',S[1,0])
        print('S*y: ',S_star[1,0])
        print(S_star + displacement_matrix - S)
    return S_star + displacement_matrix, displacement_matrix
    

def move(DIM,N,all=0):
    DeltaS_prime = np.zeros((DIM,N))

    if all:
        DeltaS_prime[:,1:] = [[np.random.normal() for _ in range(N-1)] for _ in range(DIM)]
    else:
        DeltaS_prime[:,0] = [np.random.rand(),np.random.rand()]

    return DeltaS_prime


def DM_from_platoon(platoon):
    d_mat = np.zeros((len(platoon),len(platoon)))

    for i in range(len(platoon)):
        for j in range(len(platoon)):
            d_mat[i,j] = platoon[i].get_distance(platoon[j])

    return d_mat
    

def DM_from_platoon2(platoon):
    d_mat = np.zeros((len(platoon),len(platoon)))

    for i in range(len(platoon)):
        for j in range(len(platoon)):
            d_mat[i,j] = platoon[i].get_distance(platoon[j])**2

    return d_mat


def DM_from_S2(S,verbose=0):

    e   = np.ones((1,len(S[0,:]))).T
    
    Phi_prime = np.array([np.diag(S.T@S)]).T    
    DM_prime = Phi_prime@e.T - 2*S.T@S + e@Phi_prime.T

    if verbose > 1:
        print("Phi':\n",Phi_prime)
    
    if verbose > 0:
        print("DM' :\n",DM_prime)
    
    return DM_prime


def DM_from_S(S):
    m = DM_from_S2(S)
    return np.power(m,1/2)


def noise_matrix(DIM, mu, sigma):
    m = np.zeros((DIM,DIM))

    for i in range(DIM):
        for j in range(DIM):
            if (i!=j):
                m[i,j] = np.random.normal(mu,sigma)
    return m


def square(matrix):
    return np.power(matrix,2)
    

def expected_value(matrix,noise='Gaussian'):
    if (noise == 'Gaussian'):
        for i in range(len(matrix[:,0])):
            for j in range(len(matrix[0,:])):
                matrix[i,j] = matrix[j,i] = 1/2*(matrix[i,j] + matrix[j,i])
        return matrix
    else:
        raise Exception("Different noise will be further implemented. Please assume Gaussian for now") 
    

def rotateMatrix(theta):
    return np.array([[np.cos(theta),np.sin(theta)],[-np.sin(theta),np.cos(theta)]])


#def translateMatrix(transl):
def get_theta(DM,DM_prime,S_star,displ,index=1,approx = 0,verbose=0):

    deltaX = displ[0,0]
    deltaY = displ[1,0]

    a2 = DM[0,index] - DM_prime[0,index] + deltaX**2 + deltaY**2
    b2 = -2*(S_star[0,index]*deltaX + S_star[1,index]*deltaY)         
    c2 =  2*(S_star[0,index]*deltaY - S_star[1,index]*deltaX)     

    a3 = DM[0,index+1] - DM_prime[0,index+1] + deltaX**2 + deltaY**2
    b3 = -2*(S_star[0,index+1]*deltaX + S_star[1,index+1]*deltaY)     
    c3 =  2*(S_star[0,index+1]*deltaY - S_star[1,index+1]*deltaX)     

    sinTheta = (a3*b2-a2*b3)/(b3*c2-b2*c3)
    cosTheta = (a2*c3-a3*c2)/(b3*c2-b2*c3)

    if verbose > 0:
        print("Coordinates to work with: %i-th and %i-th" % (index, index+1))

    if verbose > 1:
        print("Before normalization: ")
        print("cos(theta):\t%f" % (cosTheta))
        print("sin(theta):\t%f" % (sinTheta))
        print("After normalization: ")

    #NORMALIZATION
    mod = np.sqrt(sinTheta**2+cosTheta**2)
    sinTheta /= mod
    cosTheta /= mod

    theta = np.arctan2(sinTheta,cosTheta)
    if approx == 1:
        atheta = round(theta,4)
    elif approx == 2:
        atheta = round(theta,3)
    if verbose > 0:
        print("cos(theta):\t%f" % (cosTheta))
        print("sin(theta):\t%f" % (sinTheta))
        print("\nActual theta:\t",theta)
        
        if approx > 0:
            print("Approx theta:\t",atheta)

    if approx > 0:
        return atheta
    else:
        return theta


def MDS(S,DM,S_prime,DM_prime,S_prime2,DM_prime2,DIM=2,noise=0):

    if (noise != 0):
        DM        = expected_value(DM,noise)
        DM_prime  = expected_value(DM_prime,noise)
        DM_prime2 = expected_value(DM_prime2,noise)
 
    # Eigenvalue decomposition for a first estimation of the coordinates: S*
    S_star = EVD(DM,DIM)

    # Remove translational ambiguities
    S_star, offset = remove_offset(S,S_star)

    # Estimation of the rotation angle: theta_r
    if not noise:
        theta_r   = get_theta(DM,DM_prime,S_star,S_prime-S)
    else:
        theta_r = LSE(DM,DM_prime,S_star,S_prime-S).x

    # New rotated coordinates: S**
    S_star2 = rotateMatrix(theta_r)@S_star

    # Estimation of the new rotation angle after another displacement
    if not noise:
        theta_r2 = get_theta(DM,DM_prime2,S_star2,S_prime2-S,approx=2)
    else:
        theta_r2 = LSE(DM,DM_prime2,S_star2,S_prime2-S).x

    # Find if there is any flip ambiguities
    if (not noise) and (theta_r2 != 0):
        F = np.array([[-1,0],[0,1]])
        theta_r = get_theta(DM,DM_prime,F@S_star,S_prime-S)
        S_star2 = rotateMatrix(theta_r)@F@S_star

    if noise:
        # Let's find the optimal threshold
        l = 1/2*g(-2*np.arctan2((S_prime-S)[0,0],(S_prime-S)[1,0]) + 2*np.arctan2((S_prime2-S)[0,0],(S_prime2-S)[1,0]))
        if theta_r2 > l:
            F = np.array([[-1,0],[0,1]])
            theta_r = LSE(DM,DM_prime,F@S_star,S_prime-S).x
            S_star2 = rotateMatrix(theta_r)@F@S_star
    return S_star,S_star2


def obj(theta,DM,DM_prime,S_star,displ):
    deltaX = displ[0,0]
    deltaY = displ[1,0]

    obj = 0

    for index in range(len(DM)):

        a = DM[0,index] - DM_prime[0,index] + deltaX**2 + deltaY**2
        b = -2*(S_star[0,index]*deltaX + S_star[1,index]*deltaY)    
        c =  2*(S_star[0,index]*deltaY - S_star[1,index]*deltaX)

        obj += (a + b*np.cos(theta) + c*np.sin(theta))**2

    return obj


def LSE(DM,DM_prime,S_star,displ):

    r = minimize_scalar(obj,args=(DM,DM_prime,S_star,displ))
 
    return r























def g(t):
    if (t >= -np.pi and t < np.pi):
        return t - 2*np.pi*np.floor(t/(2*np.pi)+1/2)

def theta_i1(S_star,S,index):
    print(index)
    if index > 0:
        return g(np.arctan2(S_star[1,index],S_star[0,index])-np.arctan2(S[1,index],S[0,index]))
    else:
        print("OK")
        return None

def THETA_i(S,displ,index):
    deltaX = displ[0]
    deltaY = displ[1]
    return np.arctan2(S[1,index],S[0,index]) + np.arctan2(deltaY,deltaX) - np.pi/2

def theta_i2(theta1, THETA):
    return g(theta1 + 2*THETA) 

def estimate_theta2(DM2,DM_prime,S_star,displ,index=1,verbose=0):
    
    deltaX = displ[0]
    deltaY = displ[1]

    a = DM2[0,index] - DM_prime[0,index] + deltaX**2 + deltaY**2
    b = -2*(S_star[0,index]*deltaX + S_star[1,index]*deltaY)    
    c =  2*(S_star[0,index]*deltaY - S_star[1,index]*deltaX)    

    theta1 = np.arctan((-a*c**2-np.sqrt(-c**2*(a**2-b**2-c**2))*b)/(b**2+c**2)/c,(-a*b+np.sqrt(-c**2*(a**2-b**2-c**2)))/(b**2+c**2))
    theta2 = np.arctan((-a*c**2+np.sqrt(-c**2*(a**2-b**2-c**2))*b)/(b**2+c**2)/c,(-a*b-np.sqrt(-c**2*(a**2-b**2-c**2)))/(b**2+c**2))

    return theta1,theta2

def estimate_theta3(DM2,DM_prime,S_star,displ,index=1,verbose=0):
    
    deltaX = displ[0]
    deltaY = displ[1]

    a = DM2[0,index] - DM_prime[0,index] + deltaX**2 + deltaY**2
    b = -2*(S_star[0,index]*deltaX + S_star[1,index]*deltaY)    
    c =  2*(S_star[0,index]*deltaY - S_star[1,index]*deltaX)    

    theta1 = np.arctan2(a*b + np.abs(c)*np.sqrt(b**2+c**2-a**2),a*c - b/c*np.abs(c)*np.sqrt(b**2+c**2-a**2))
    theta2 = np.arctan2(a*b - np.abs(c)*np.sqrt(b**2+c**2-a**2),a*c + b/c*np.abs(c)*np.sqrt(b**2+c**2-a**2))

    return theta1,theta2
