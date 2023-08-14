#!/usr/bin/env python3

import numpy as np
# from scipy.optimize import least_squares
from scipy.optimize import minimize


def noise(mean=0, std=0.01, shape=[3, 1]):
    if type(shape) == int:
        return np.random.normal(mean, std, size=[shape, shape])
    return np.random.normal(mean, std, size=shape)

def get_distance(point1: np.array, point2: np.array):
    """
    Compute the distance between two points, i.e. compute vectorial difference.
    The difference is computed via scalar product and Carnot theorem

    It must be used for the - 1 - displacement
    """
    square_distance = np.sum((point1 - point2)**2)

    return [square_distance]


def distance_matrix(X):
    """
    Compute the distance matrix given the coordinates matrix.
    In reality, the distances are retrieved from UWB sensors
    """

    # Unitary array
    e = np.ones(X.shape[1]).reshape(-1, 1)

    Phi = np.diag(X.T @ X).reshape(-1, 1)
    D = Phi @ e.T - 2 * X.T @ X + e @ Phi.T

    return D


def move_anchor(points, step=0, displacement=1):
    """
    Given the set of points, make the anchor move for a given displacement.
    The step value indicates the phase/update of the anchor
    """
    motion = np.zeros(points.shape)

    if (step):
        if (step == 1):
            motion[0, 0] += displacement
        elif (step == 2):
            motion[0, 0] -= displacement
            motion[1, 0] += displacement
        elif (step == 3):
            motion[1, 0] -= displacement
            motion[2, 0] += displacement
        elif (step == 4):
            motion[2, 0] -= displacement

    # Apply the displacement
    points = points + motion

    return points[:, 0].reshape(3, -1), points


def move_fleet(points, low=0, high=0):
    """
    Given the set of points, make the anchor move for a given displacement.
    The step value indicates the phase/update of the anchor
    """
    motion = np.zeros(points.shape)

    motion[:, 1:] += np.random.uniform(low=low, high=high,
                                       size=[points.shape[0], points.shape[1] - 1])

    # Apply the displacement
    points = points + motion

    return points


def combine_matrices(D1, D2, D3, D4, P1, P2, P3, P4):

    # The matrices might be asymmetric due to noise. Use instead the expected values to overcome this
    D1 = 1/2*(D1 + D1.T)
    D2 = 1/2*(D2 + D2.T)
    D3 = 1/2*(D3 + D3.T)
    D4 = 1/2*(D4 + D4.T)

    # 1 - Start from the initial distance matrix D1
    DM = D1.copy()

    # 2 - Add the second distance matrix information
    # Create the new column vector to add to the matrix
    col = np.hstack([get_distance(P1, P2), D2[0, 1:],
                    get_distance(P2, P2)]).reshape(-1, 1)

    # Add the vector to the symmetric matrix
    DM = np.hstack([DM, col[:-1]])
    DM = np.vstack([DM, col.reshape(1, -1)])

    # 3 - Add the second distance matrix information
    # Create the new column vector to add to the matrix
    col = np.hstack([get_distance(P1, P3), D3[0, 1:], get_distance(
        P2, P3), get_distance(P3, P3)]).reshape(-1, 1)

    # Add the vector to the symmetric matrix
    DM = np.hstack([DM, col[:-1]])
    DM = np.vstack([DM, col.reshape(1, -1)])

    # 4 - Add the second distance matrix information
    # Create the new column vector to add to the matrix
    col = np.hstack([get_distance(P1, P4), D4[0, 1:], get_distance(
        P2, P4), get_distance(P3, P4), get_distance(P4, P4)]).reshape(-1, 1)

    # Add the vector to the symmetric matrix
    DM = np.hstack([DM, col[:-1]])
    DM = np.vstack([DM, col.reshape(1, -1)])

    return DM


def sphere(x, x_a, d):
    return (x[0] - x_a[0])**2 + (x[1] - x_a[1])**2 + (x[2] - x_a[2])**2 - d


def LS(distance_matrix, anchor_pos):
    """
    distance_matrix:
    X : anchor coords that are known

    Function implementing trilateration.
    distances: matrix of distances measured between the i-th anchor and the n node
            It is structured as following:
            matrix 4xN, of which each column contains the distance from the i-th anchor
    """

    distances = np.vstack([distance_matrix[0, 1:-3], distance_matrix[-3, 1:-3],
                          distance_matrix[-2, 1:-3], distance_matrix[-1, 1:-3]])

    assert len(distances) == 4

    N = distances.shape[1]
    X_hat = np.zeros((3, 1+N))

    def f(variables, anchor_pos, d):
        x, y, z = variables
        obj = 0

        for j in range(4):
            dx = x - anchor_pos[0, j]
            dy = y - anchor_pos[1, j]
            dz = z - anchor_pos[2, j]
            obj += (dx**2 + dy**2 + dz**2 - d[j])**2

        return obj

    for i in range(N):
        sol = minimize(f, np.ones((3,)), args=(anchor_pos,  distances[:, i]))

        X_hat[:, 1+i] = sol.x

    X_hat[:, 0] = anchor_pos[:, 0]
    return X_hat


def EVD(DM, final_dimension):
    """
    Return the relative map of the nodes in the network.
    """
    # Current size of the matrix. It is square, thus len = shape
    n = len(DM)

    # Centering Matrix definition
    H = np.eye(n) - np.ones((n, n))/n

    # Double centered matrix
    B = -1/2*H@DM@H

    # Eiegenvalue decomposition
    ev, EV = np.linalg.eig(B)

    # Sort according to the eigenvalues magnitude
    seq = np.argsort(ev)[::-1]

    LAMBDA = np.diag(ev[seq][:final_dimension])
    U = EV[:, seq][:, :final_dimension]

    # Relative map
    X1 = np.sqrt(LAMBDA)@U.T

    return X1.real


def MDS(distance_matrix, anchor_pos, true_pos=None):
    """
    Compute the actual coordinates of the points given the complete distance matrix.
    The matrix must contain n+1 known coordinates, i.e. n+1 anchors
    """

    # Compute the relative map of the points
    S = EVD(distance_matrix, 3)

    # Define P and Q, two sets of corresponding nodes
    # P = anchors from the relative map
    # Q = true position of the anchors

    p = np.hstack([S[:, 0].reshape(-1, 1), S[:, -3:]])
    q = anchor_pos

    p_bar, q_bar = 1/4*np.sum(p, axis=1).reshape(-1, 1), 1 / \
        4*np.sum(q, axis=1).reshape(-1, 1)
    P_prime = p - p_bar
    Q_prime = q - q_bar

    H = P_prime @ Q_prime.T

    # UU is returned correctly but has to be used transposed
    # VV is returned transposed but has to be used normally
    UU, SS, VV = np.linalg.svd(H)

    R = VV.T @ UU.T

    t = q_bar - R @ p_bar

    X_hat = R @ S + t

    return X_hat[:, :-3]

    # Numerical minimization with surrogate model
    # # # P = anchor_pos
    # # # P_prime = np.hstack([S[:,0].reshape(-1,1), S[:,-3:]])
    # # # def f(x):
    # # #     R, t = to_R_t(x)

    # # #     out = 0
    # # #     for i in range(P.shape[1]):
    # # #         out += np.sum( ( R @ P_prime[:,i] + t - P[:,i])**2 )
    # # #         # (x-x0) W (x-xo)^T

    # # #     return out

    # # # x0 = np.hstack([np.ones([1,9]), np.ones([1,3])])

    # # # v = minimize(f, x0, method='BFGS' )

    # # # Rp, tp = to_R_t(v.x)

    # # # X_hat = Rp @ S + tp

    # # # return X_hat[:,:-3]


# def to_vector(R, t):
#     return np.hstack([R.flatten(), t.flatten()])


def to_R_t(vec):
    vec = vec.reshape(12,)
    return vec[:3*3].reshape(3,3), vec[3*3:].reshape(3,-1)



def MDS_2(distance_matrix, anchor_pos, true_pos=None):
    """
    Compute the actual coordinates of the points given the complete distance matrix.
    The matrix must contain n+1 known coordinates, i.e. n+1 anchors
    """

    # Compute the relative map of the points
    S = EVD(distance_matrix, 3)

    P_prime = np.hstack([S[:,0].reshape(-1,1), S[:,-3:]])
    def f(x, P_bar):
        R, t, P = to_R_t_2(x)

        out = 0
        for i in range(P.shape[1]):
            out += np.sum( ( R @ P_prime[:,i] + t - P[:,i])**2 )
            out += np.sum( ( P[:,i] - P_bar[:,i])**2 )            

        return out**2

    x0 = np.hstack([
        np.eye(3).reshape(1,-1), 
        np.zeros([1,3]),
        anchor_pos.reshape(1,-1)
    ])

    v = minimize(f, x0, method='BFGS', args=(anchor_pos))

    Rp, tp, Pp = to_R_t_2(v.x)

    X_hat = Rp @ S + tp

    return X_hat[:,:-3]


# def to_vector(R, t):
#     return np.hstack([R.flatten(), t.flatten()])


def to_R_t_2(vec):
    vec = vec.reshape(24,)
    return vec[:9].reshape(3,3), vec[9:12].reshape(3,-1), vec[12:].reshape(3,-1)

# def M_ROT_TRASL_DRONE_GZ(i): return np.eye(4)
def M_ROT_TRASL_DRONE_GZ(i): 
    return np.array([[0, 1, 0, i], [-1, 0, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]])


# M_ROT_TRASL_GZ_DRONE = MatrixInverse(M_ROT_TRASL_Z_DRONE_GZ)
# def M_ROT_TRASL_GZ_DRONE(i): return np.eye(4)
def M_ROT_TRASL_GZ_DRONE(i): return np.array(
    [[0, -1, 0, 0], [1, 0, 0, -i], [0, 0, 1, 0], [0, 0, 0, 1]])