import numpy as np
import numpy.matlib
from scipy.stats import multivariate_normal
from sklearn.manifold import SpectralEmbedding

# from vrepVS060Ctrl.densoVS060kine import densoVS060Kine
from motionOpt.IWVBEM import IWVBEMGMMfitting
from motionOpt.cost_function import computeCost
from motionOpt.CHOMP import chomp

def multiTrajOpt(robKine, traj_q, T, ite_num, obstacles, radii, eps, body_sizes, costweights,
                 sampleNum=500, collision_threshold=1.5, MaxSol=10, scale=20, seed_num=0):

    np.random.seed(seed_num)
    link_num = traj_q.shape[1]
    K = np.diag([1] * T, 0) + np.diag([-1] * (T-1), -1)
    K[0, 0] = 0
    K[1, 0] = 0

    A = K.T @ K
    A[-1, -1] = 2

    B = np.copy(A[:-1, 1:-1])
    R = np.linalg.inv(B.T @ B)

    zero_mean = np.zeros((T-2,))

    N = sampleNum
    traj_sample_set = np.zeros((T, link_num, N))
    cost_set = np.zeros((N, 1))

    p_set = np.zeros((N, 1))
    zero_ini = np.zeros((1, link_num))

    p_ini = 1
    traj_hto = []
    modeNum = 1
    explr_schedule = 1

    for iteration in range(ite_num):
        explr_schedule = 1 - 0.5*iteration/ite_num
        M = explr_schedule * 3.0 * R / (R.max() * T)
        print('Start sampling.')
        for k in range(N):
            noise = np.random.multivariate_normal(zero_mean, M, link_num)
            p = multivariate_normal.pdf(noise, mean=zero_mean, cov=M)

            if iteration == 0 and k== 0:
                p_ini = p[0]
            p_set[k] = np.prod(p / p_ini)
            traj_noise = np.vstack([zero_ini, np.transpose(noise), zero_ini])

            if iteration == 0:
                traj_q_noise = traj_q + traj_noise
            else:
                m = np.mod(k, modeNum)
                traj_q_noise = traj_hto[m,:,:] + traj_noise

            cost_set[k, 0], _, _ = computeCost(robKine, traj_q_noise, body_sizes, obstacles, radii, eps, costweights)
            traj_sample_set[:,:,k] = traj_q_noise

        D = []
        for k in range(N):
            D.append(np.reshape(traj_sample_set[:, :, k], (T * link_num,)))

        exp_arg = - scale * (cost_set - cost_set.min()) / (cost_set.max() - cost_set.min())
        exp_cost_set = np.exp(exp_arg)
        exp_cost_set = exp_cost_set / exp_cost_set.mean()
        # print('exp_cost_set', exp_cost_set)

        print('performing dimensionality reduction...')
        embedding = SpectralEmbedding(n_components=2)
        D_transformed = embedding.fit_transform(np.asarray(D))
        max_D = np.max(D_transformed)
        D_transformed = D_transformed /max_D
        # print('D_transformed', D_transformed)
        print('fitting GMM...')

        z, modeNum = IWVBEMGMMfitting(D_transformed, exp_cost_set, 1000, MaxSol)
        z = np.reshape(z, (N,))
        # print(z)
        print('...done')

        traj_hto = np.zeros((modeNum, T, link_num))
        cost_m = np.zeros((modeNum, 3))

        for m in range(modeNum):
            ind_m = ( z== m )
            traj_sample_set_m =  np.asarray(D)[ind_m, :]

            num_m = np.sum(ind_m)
            print('num_m', num_m)
            weight_m = exp_cost_set[ind_m, :]

            weight_m_2d = np.matlib.repmat(weight_m, 1, T*link_num)
            mean_traj_m = np.sum( np.multiply(weight_m_2d, traj_sample_set_m), axis=0) / np.sum(weight_m)

            traj_m = np.reshape(mean_traj_m, (T, link_num))

            traj_m = chomp(robKine, traj_m, T, obstacles, radii, eps, body_sizes, costweights,
                              MaxIter=10, update_rate=0.04)

            traj_hto[m, :, :] = np.reshape(mean_traj_m, (T, link_num))

            cost, collision, smoothness = computeCost(robKine, traj_m, body_sizes, obstacles, radii, eps, costweights)
            print('total: ', cost, 'collision: ', collision, 'smoothness: ', smoothness)
            cost_m[m, 0] = cost
            cost_m[m, 1] = collision
            cost_m[m, 2] = smoothness

        check_thre = sum( cost_m[:, 1] < collision_threshold)
        print('number of solutions that satisfy the collision threshold: ', check_thre)
        if check_thre > 1:
            print('break')
            break

    return traj_hto, modeNum, cost_m







