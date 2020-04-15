import numpy as np
import numpy.matlib
from scipy.linalg import null_space
from scipy.stats import multivariate_normal
from sklearn.manifold import SpectralEmbedding

# from vrepVS060Ctrl.densoVS060kine import densoVS060Kine
from motionOpt.IWVBEM import IWVBEMGMMfitting
from motionOpt.cost_function import computeCost
from motionOpt.CHOMP import chompend

def multiTrajOptEnd(robKine, traj_q, T, ite_num, obstacles, radii, eps, body_sizes, stomp_weights, angle_pos, angle_neg,
                    freeAxis, jointLimit_low, jointLimit_high, dim=3,
                    sampleNum=500, update_rate_chomp=0.04, opt_threshold = 2.0,
                    MaxSol=10, scale=20, seed_num=0):

    np.random.seed(seed_num)
    link_num = traj_q.shape[1]

    K = np.diag([1] * T, 0) + np.diag([-1] * (T-1), -1)
    K[0, 0] = 0
    K[1, 0] = 0

    A = K.T @ K
    A[-1, -1] = 2

    M_end = np.linalg.pinv(A)
    B = np.copy(A[:-1, 1:-1])
    R = np.linalg.inv(B.T @ B)

    zero_mean = np.zeros((T-2,))

    N = sampleNum
    g_seedNum = 400

    traj_sample_set = np.zeros((T, link_num, N))
    cost_set = np.zeros((N, 1))

    p_set = np.zeros((N, 1))
    zero_ini = np.zeros((1, link_num))

    p_ini = 1
    traj_hto = []
    modeNum = 1

    goal_config_given = traj_q[T-1,:]
    Nend = np.floor(g_seedNum * angle_pos / (angle_pos + angle_neg))
    goal_config = np.copy(goal_config_given)
    goal_task = np.copy(robKine.computeEndPosConfig(goal_config_given * 180/np.pi))

    # Prepare a set of possible goal configurations
    g_config_set = np.zeros((g_seedNum + 1, link_num))
    g_config_set[0,:] =  goal_config_given

    for i in range(1, g_seedNum):
        if i < Nend + 1:
            noise_ang = angle_pos / Nend
        else:
            noise_ang = angle_neg / (g_seedNum - Nend)

        noise_endRot = noise_ang * freeAxis
        noise_endRot = np.hstack([np.zeros((3, )), noise_endRot])

        robKine.computeJacobianConfig(goal_config.flatten()*180/np.pi)
        dq = robKine.ComputeJointVelfromCartVel(np.reshape(noise_endRot, (6,1)))

        if i < Nend + 1:
            goal_config = goal_config.flatten() - dq.flatten()
        else:
            goal_config = goal_config.flatten() + dq.flatten()

        if np.greater(jointLimit_low, goal_config).any() or np.greater(goal_config, jointLimit_high).any():
            print('goal_config', goal_config)
            print('joint limit, i: ',  i)
            break

        for ite in range(10):
            goal_config_in = goal_config.flatten()
            x, J = robKine.computeJacobianEndPosConfig(goal_config_in*180/np.pi)

            goal_now = np.zeros((1, 3))
            goal_now[:] = x
            goal_error = goal_task - goal_now

            if dim == 2:
                goal_error = np.vstack([goal_error.transpose(), np.zeros((4, 1))])
            else:
                goal_error = np.vstack([goal_error.transpose(), np.zeros((3, 1))])

            goal_config = goal_config + 0.2 * np.transpose(robKine.ComputeJointVelfromCartVel(goal_error))

            if np.linalg.norm(goal_error) < 0.01:
                break

        g_config_set[i,:] =  goal_config

        if i == Nend:
            goal_config = goal_config_given

    g_config_set = g_config_set[g_config_set.any(axis=1), :]

    D_transformed = []
    exp_cost_set = []
    z = []
    epsilon = 0.4

    for iteration in range(ite_num):
        explr_schedule = 1
        if ite_num > 1:
            explr_schedule = 1 - 0.5*iteration/(ite_num-1)

        M = explr_schedule * 3.0 * R / (R.max() * T)

        print('sampling trajectories...')
        for k in range(N):
            noise = np.random.multivariate_normal(zero_mean, M, link_num)
            p = multivariate_normal.pdf(noise, mean=zero_mean, cov=M)

            if iteration == 0 and k== 0:
                p_ini = p[0]
            p_set[k] = np.prod(p / p_ini)

            traj_noise = np.vstack([zero_ini, np.transpose(noise), zero_ini])

            traj_q_noise = []
            if iteration == 0:
                g_noise_ind = np.random.randint(low=0, high=g_config_set.shape[0]-1)
                g_noise_traj = np.zeros((T, link_num))

                if link_num > 6:
                    g_noise_config = g_config_set[g_noise_ind, :]
                    J = robKine.computeJacobianConfig(g_noise_config.flatten()*180/np.pi)
                    vector_null = null_space(J)
                    g_noise_traj[T - 1, :] = g_config_set[g_noise_ind, :] - goal_config_given + np.random.uniform(- epsilon, epsilon)* np.reshape(vector_null, (1, link_num))
                else:
                    g_noise_traj[T - 1, :] = g_config_set[g_noise_ind, :] - goal_config_given

                g_noise_prop = np.dot(M_end, g_noise_traj)

                traj_q_noise = traj_q + traj_noise + g_noise_prop
            else:

                m = np.mod(k, modeNum)
                if link_num > 6:
                    g_config_m = traj_hto[m,T-1,:]
                    J = robKine.computeJacobianConfig(g_config_m.flatten() * 180 / np.pi)
                    vector_null = null_space(J)
                    null_noise_traj = np.zeros((T, link_num))
                    null_noise_traj[T - 1, :] = np.random.uniform(-epsilon, epsilon) * np.reshape(vector_null, (1, link_num))
                    null_noise_prop = np.dot(M_end, null_noise_traj)
                    traj_q_noise = traj_hto[m, :, :] + traj_noise + null_noise_prop
                else:
                    traj_q_noise = traj_hto[m,:,:] + traj_noise

            cost_set[k, 0], _, _ = computeCost(robKine, traj_q_noise, body_sizes, obstacles, radii, eps, stomp_weights)
            traj_sample_set[:,:,k] = traj_q_noise


        D = []
        for k in range(N):
            D.append( np.reshape(traj_sample_set[:,:,k], (T*link_num, )) )

        exp_arg = - scale * (cost_set - cost_set.min()) / (cost_set.max() - cost_set.min())
        exp_cost_set = np.exp(exp_arg)
        exp_cost_set = np.divide(np.exp(exp_arg), p_set)
        exp_cost_set = exp_cost_set / exp_cost_set.mean()

        print('performing dimensionality reduction...')
        embedding = SpectralEmbedding(n_components=3)
        D_transformed = embedding.fit_transform(np.asarray(D))
        max_D = np.max(D_transformed)
        D_transformed = D_transformed /max_D
        # print('D_transformed', D_transformed)

        print('fitting GMM...')
        z, modeNum = IWVBEMGMMfitting(D_transformed, exp_cost_set, 1000, MaxSol)
        z = np.reshape(z, (N,))
        # print(z)

        traj_hto = np.zeros((modeNum, T, link_num))
        cost_m = np.zeros((modeNum, 3))
        print('goal_task', goal_task)

        chomp_ite = 30
        if ite_num > 1:
            opt_thre =  3.0 + (opt_threshold - 3.0 ) * iteration / (ite_num-1)
            chomp_ite = 15
        for m in range(modeNum):
            ind_m = ( z== m )
            traj_sample_set_m =  np.asarray(D)[ind_m, :]

            num_m = np.sum(ind_m)
            print('num of samples in this mode', num_m)
            weight_m = exp_cost_set[ind_m, :]

            weight_m_2d = np.matlib.repmat(weight_m, 1, T*link_num)
            mean_traj_m = np.sum( np.multiply(weight_m_2d, traj_sample_set_m), axis=0) / np.sum(weight_m)

            traj_m = np.reshape(mean_traj_m, (T, link_num))
            goal_error = []

            traj_m = chompend(robKine, goal_task, traj_m, T, obstacles, radii, eps, body_sizes, stomp_weights, freeAxis,
                     MaxIter=chomp_ite,  update_rate=update_rate_chomp, collision_threshold=opt_thre)

            cost, collision, smoothness = computeCost(robKine, traj_m, body_sizes, obstacles, radii, eps, stomp_weights)
            print('total: ', cost, 'collision: ', collision, 'smoothness: ', smoothness)
            cost_m[m, 0] = cost
            cost_m[m, 1] = collision
            cost_m[m, 2] = smoothness

            traj_hto[m, :, :] = traj_m

        check_thre = sum( cost_m[:, 1] < opt_threshold)
        print('number of solutions that satisfy the collision threshold: ', check_thre)
        if check_thre > 1:
            print('break')
            break


    # remove the solutions that exceed the threshold of the collision cost
    ind_ok = ( cost_m[:, 1] < 7.0)
    modeNum = int(sum( cost_m[:, 1] < 7.0))
    traj_hto = traj_hto[ind_ok, :, :]
    cost_m = cost_m[ind_ok, :]

    return traj_hto, modeNum, cost_m, D_transformed, exp_cost_set, z







