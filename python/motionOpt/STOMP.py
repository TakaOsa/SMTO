import numpy as np
import numpy.matlib
from scipy.stats import multivariate_normal
from sklearn.manifold import SpectralEmbedding
from scipy.linalg import null_space

from motionOpt.IWVBEM import IWVBEMGMMfitting
from motionOpt.cost_function import computeCost, computeCostGradientPoint, computeCollisionCostPoint, traj_q2t, computeCollisionCost


def stomp(robKine, traj_q, T, obstacles, radii, eps, body_sizes, costweights, ite_num=10, noise_array=None, collision_threshold=1.5):
    link_num = traj_q.shape[1]

    K = np.diag([1] * T, 0) + np.diag([-1] * (T - 1), -1)
    K[0, 0] = 0
    K[1, 0] = 0

    A = K.T @ K
    A[-1, -1] = 2

    B = np.copy(A[:-1, 1:-1])
    R = np.linalg.inv(B.T @ B)

    M = R / (R.max() * T)
    zero_mean = np.zeros((T - 2,))

    N = 5

    noise_set = np.zeros((T - 2, link_num, N))
    cost_set = np.zeros((N, 1))
    best_traj_sample_set = np.zeros((T, link_num, N))
    best_traj_costs = 100 * np.ones((N, 1))

    ## REFACTOR THIS FOR LOOP
    cost = 0
    collision = 0
    smoothness= 0
    for iteration in range(ite_num):
        if iteration > 0:
            for k in range(N):
                if cost_set[k, 0] < best_traj_costs.max():
                    ind = best_traj_costs.argmax()
                    best_traj_costs[ind, 0] = cost_set[k, 0]
                    traj_noise = np.vstack((np.zeros((1, link_num)), noise_set[:, :, k], np.zeros((1, link_num))))
                    best_traj_sample_set[:, :, ind] = traj_q + traj_noise

        for k in range(N):
            if noise_array is None:
                noise = np.random.multivariate_normal(zero_mean, M, link_num)
            else:
                noise = noise_array[iteration, k, :, :].squeeze()

            traj_noise = np.vstack((np.zeros((1, link_num)), noise.T, np.zeros((1, link_num))))
            traj_q_noise = traj_q + traj_noise
            noise_set[:, :, k] = noise.T
            cost_set[k, 0], _, _ = computeCost(robKine, traj_q_noise, body_sizes, obstacles, radii, eps, costweights)

        if iteration > 0:
            cost_set = np.vstack((cost_set, best_traj_costs))
            best_noise_batch = best_traj_sample_set - traj_q[:, :, None]
            best_noise_batch = best_noise_batch[1:-1, :, :]
            noise_set = np.concatenate((noise_set, best_noise_batch), axis=2)

        exp_arg = - 10 * (cost_set - cost_set.min()) / (cost_set.max() - cost_set.min())
        exp_cost_set = np.exp(exp_arg)
        exp_cost_set = exp_cost_set / exp_cost_set.sum()

        delta_q = np.zeros((T - 2, link_num))
        for k in range(N):
            delta_q[:, :] = delta_q[:, :] + exp_cost_set[k, 0] * noise_set[:, :, k]

        update_q = np.vstack((np.zeros((1, link_num)), M @ delta_q, np.zeros((1, link_num))))
        traj_q = traj_q + update_q

        cost, collision, smoothness = computeCost(robKine, traj_q, body_sizes, obstacles, radii, eps, costweights)
        if collision < collision_threshold:
            print('cost is smaller than the threshold')
            print('total: ', cost, 'collision: ', collision, 'smoothness: ', smoothness)
            break


    print('cost_set_max: %f, update num: %d \n' % (cost_set.max(), iteration))
    print('total: ', cost, 'collision: ', collision, 'smoothness: ', smoothness)

    return traj_q









