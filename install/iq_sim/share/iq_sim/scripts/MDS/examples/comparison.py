import sys, time, ast
sys.path.append('')

import numpy as np
from UAV import *
import matplotlib.pyplot as plt

np.random.seed(10)


def comparison(parameters):
    # read the parametersc
    if parameters['noise']:
        sim_function = simulate_noise
    else:
        sim_function = simulate

    number_experiments = parameters['number_experiments']

    # prepare result arrays
    mean_dist_MDS = np.zeros((3, number_experiments))
    mean_dist_WLP = np.zeros((3, number_experiments))

    std_dist_MDS = np.zeros((3, number_experiments))
    std_dist_WLP = np.zeros((3, number_experiments))

    # run experiments
    for i in range(parameters['number_experiments']):
        # print("Experiment: %s" % str(i))
        X, X_hat_MDS, X_hat_WLP = sim_function(parameters)
        mean_dist_MDS[:, i] = np.sum(X-X_hat_MDS, axis=1)
        mean_dist_WLP[:, i] = np.sum(X-X_hat_WLP, axis=1)

        std_dist_MDS[:, i] = np.mean(np.abs(X-X_hat_MDS), axis=1)
        std_dist_WLP[:, i] = np.mean(np.abs(X-X_hat_WLP), axis=1)

    # verify performances
    # TODO: better define the metrics to understand the performances, these are not the matrices the professor was talking about
    print("Bias verification")
    print("MDS: %s" % str(np.sum(mean_dist_MDS, axis=1)))
    print("WLP: %s" % str(np.sum(mean_dist_WLP, axis=1)))

    print("Variance verification")
    print("MDS: %s" % np.mean(std_dist_MDS, axis=1))
    print("WLP: %s" % np.mean(std_dist_WLP, axis=1))



# python examples/comparison.py number_experiments=100 number_uavs=4
if __name__ == '__main__':

    # Default simulation parameters
    params = {'number_uavs': 20, 'noise': 'gaussian'} # 'noise': 'gaussian' # eventually None

    # Load parameter from terminal, if provided
    for arg in sys.argv[1:]:
        key, value = arg.split('=')
        try:
            value = ast.literal_eval(str(value))

            if (key in ['number_experiments', 'number_uavs']):
                params[key] = value
            else:
                print("Inserted parameter not recognised.")
                exit(1)
        except (SyntaxError, ValueError) as e:
            print("LOG: ", e)
            exit(1)

    # Start the simulation
    comparison(parameters=params)
