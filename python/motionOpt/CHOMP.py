import numpy as np
import numpy.matlib
from scipy.stats import multivariate_normal
from sklearn.manifold import SpectralEmbedding
from scipy.linalg import null_space

from motionOpt.IWVBEM import IWVBEMGMMfitting
from motionOpt.cost_function import computeCost, computeCostGradientPoint, computeCollisionCostPoint, traj_q2t, computeCollisionCost


def chomp(robKine, traj_q, T, obstacles, radii, eps, body_sizes, chomp_weights, MaxIter=10, dim=3, collision_threshold=1.5, update_rate=0.01):

    K = np.diag([1] * T, 0) + np.diag([-1] * (T-1), -1)
    K[0, 0] = 0
    K[1, 0] = 0

    A = K.T @ K
    A[-1, -1] = 2
    pinv_A = np.linalg.pinv(A)

    I = np.eye(dim)

    link_num = traj_q.shape[1]

    traj_task = np.zeros((T, link_num+1, dim))
    vel_task = np.zeros((T, link_num+1, dim))
    accel_task = np.zeros((T, link_num+1, dim))

    J_traj = np.zeros((T, dim, link_num))

    cost_traj = np.zeros((T, link_num+1))
    cost_gradient_traj = np.zeros((T, link_num+1, dim))

    F_grad_obs = np.zeros((T, link_num))
    F_grad_smooth = np.zeros((T, link_num))

    collision_cost_prev = -1
    cnt = 0

    for t in range(T):
        x, J = robKine.computeLinkPosJacob(traj_q[t, :] * 180 / np.pi)
        traj_task[t, :, :] = x
        J_traj[t, :, :] = J

    while True:
        cnt = cnt + 1
        for t in range(T):
            x, J = robKine.computeLinkPosJacob(traj_q[t, :] * 180 / np.pi)
            traj_task[t, :, :] = x
            J_traj[t, :, :] = J

        for t in range(1,T):
            vel_task[t, :, :] = traj_task[t, :, :] - traj_task[t-1, :, :]
            accel_task[t, :, :] = vel_task[t, :, :] - vel_task[t-1, :, :]

        for b in range(link_num+1):
            for t in range(T):
                body_point = traj_task[None, t, b, :]
                cost_traj[t, b] = computeCollisionCostPoint(body_point, body_sizes[b], obstacles, radii, eps)
                cost_gradient_traj[t, b, :] = computeCostGradientPoint(body_point, body_sizes[b], obstacles, radii, eps)

        collision_cost = np.sum(cost_traj)

        boundary = np.zeros((T, link_num))
        boundary[1, :] = - traj_q[0, :]

        b_end = link_num

        vel = np.copy(vel_task[:, b_end, :])
        accel = np.copy(accel_task[:, b_end, :])

        F_grad_smooth[:, :] = K.T @ K @ traj_q + boundary

        for t in range(1, T - 1):
            __, J = robKine.computeLinkPosJacob(traj_q[t, :]*180/np.pi)

            norm_vel = np.linalg.norm(vel[t, :])
            projection = I - vel[None, t, :].T @ (vel[None, t, :]) / (norm_vel ** 2)
            Ka = projection @ (accel[None, t, :].T) / (norm_vel ** 2)
            F_grad_obs[t, :] = norm_vel * J.T @ (projection.dot(cost_gradient_traj[t, b_end, :, None]) - cost_traj[t, b_end] * Ka).squeeze(1)


        update = pinv_A @ (chomp_weights[0] * F_grad_obs - chomp_weights[1] * F_grad_smooth)
        # print('update', update)
        traj_q = traj_q - update_rate * update

        # if np.abs(collision_cost - collision_cost_prev) < 1e-3:
        #     print('CHOMP converged.')
        #     break

        collision_cost_prev = collision_cost

        if collision_cost < collision_threshold:
            print('Collision cost smaller than threshold.')
            break

        if cnt > MaxIter:
            print('Reached the max number of iteration.')
            break


    print('collision_cost: %f, update num: %d \n' % (collision_cost, cnt))
    cost_m = np.zeros(3)
    cost, collision, smoothness = computeCost(robKine, traj_q, body_sizes, obstacles, radii, eps, chomp_weights)
    # print('total: ', cost, 'collision: ', collision, 'smoothness: ', smoothness)
    cost_m[0] = cost
    cost_m[1] = collision
    cost_m[2] = smoothness

    return traj_q


def chompend(robKine, goal_task, traj_q, T, obstacles, radii, eps, body_sizes, chomp_weights, freeAxis, MaxIter=10, dim=3,
             jointLimit_low = - 3*np.pi, jointLimit_high = 3*np.pi, collision_threshold=3.0, update_rate=0.01):

    K = np.diag([1] * T, 0) + np.diag([-1] * (T-1), -1)
    K[0, 0] = 0
    K[1, 0] = 0

    A = K.T @ K
    A[-1, -1] = 2
    pinv_A = np.linalg.pinv(A)

    I = np.eye(dim)

    link_num = traj_q.shape[1]

    traj_task = np.zeros((T, link_num+1, dim))
    vel_task = np.zeros((T, link_num+1, dim))
    accel_task = np.zeros((T, link_num+1, dim))

    J_traj = np.zeros((T, dim, link_num))

    cost_traj = np.zeros((T, link_num+1))
    cost_gradient_traj = np.zeros((T, link_num+1, dim))

    F_grad_obs = np.zeros((T, link_num))
    F_grad_smooth = np.zeros((T, link_num))

    collision_cost_prev = -1
    cnt = 0

    for t in range(T):
        x, J = robKine.computeLinkPosJacob(traj_q[t, :] * 180 / np.pi)
        traj_task[t, :, :] = x
        J_traj[t, :, :] = J

    while True:
        cnt = cnt + 1
        for t in range(T):
            x, J = robKine.computeLinkPosJacob(traj_q[t, :] * 180 / np.pi)
            traj_task[t, :, :] = x
            J_traj[t, :, :] = J

        for t in range(1,T):
            vel_task[t, :, :] = traj_task[t, :, :] - traj_task[t-1, :, :]
            accel_task[t, :, :] = vel_task[t, :, :] - vel_task[t-1, :, :]

        for b in range(link_num+1):
            for t in range(T):
                body_point = traj_task[None, t, b, :]
                cost_traj[t, b] = computeCollisionCostPoint(body_point, body_sizes[b], obstacles, radii, eps)
                cost_gradient_traj[t, b, :] = computeCostGradientPoint(body_point, body_sizes[b], obstacles, radii, eps)

        collision_cost = np.sum(cost_traj)

        boundary = np.zeros((T, link_num))
        boundary[1, :] = - traj_q[0, :]

        b_end = link_num

        vel = np.copy(vel_task[:, b_end, :])
        accel = np.copy(accel_task[:, b_end, :])

        F_grad_smooth[:, :] = K.T @ K @ traj_q + boundary

        for t in range(1, T - 1):
            __, J = robKine.computeLinkPosJacob(traj_q[t, :]*180/np.pi)

            norm_vel = np.linalg.norm(vel[t, :])
            projection = I - vel[None, t, :].T @ (vel[None, t, :]) / (norm_vel ** 2)
            Ka = projection @ (accel[None, t, :].T) / (norm_vel ** 2)
            F_grad_obs[t, :] = norm_vel * J.T @ (projection.dot(cost_gradient_traj[t, b_end, :, None]) - cost_traj[t, b_end] * Ka).squeeze(1)

        update = pinv_A @ (chomp_weights[0] * F_grad_obs - chomp_weights[1] * F_grad_smooth)
        traj_q = traj_q - update_rate * update

        # Shift the trajectory for adapting the goal position
        goal_error_norm = 1.0
        l = 0

        d_ang = 0.02
        for r in range(1):
            goal_config = traj_q[T - 1, :]
            task_pos_now = traj_q2t(robKine, np.reshape(goal_config, (1, link_num)))
            collision_cost_now = computeCollisionCost(task_pos_now, body_sizes, obstacles, radii, eps)

            noise_endRot = d_ang * freeAxis
            noise_endRot = np.hstack([np.zeros((3,)), noise_endRot])

            robKine.computeJacobianConfig(goal_config.flatten() * 180 / np.pi)
            dq = robKine.ComputeJointVelfromCartVel(np.reshape(noise_endRot, (6, 1)))
            goal_config_dq = goal_config.flatten() + dq.flatten()
            task_pos_ang = traj_q2t(robKine, np.reshape(goal_config_dq, (1, link_num)))
            collision_cost_dq = computeCollisionCost(task_pos_ang, body_sizes, obstacles, radii, eps)

            if collision_cost_dq == collision_cost_now:
                break
            else:
                rot_update_traj = np.zeros((T, link_num))
                rot_update_traj[T - 1, :] = dq.flatten() * np.sign(collision_cost_now - collision_cost_dq)
                rot_update_prop = np.dot(pinv_A, rot_update_traj)
                traj_q = traj_q + rot_update_prop

        if link_num > 6:
            for i in range(20):
                g_config_i = traj_q[T - 1, :]
                task_pos_i = traj_q2t(robKine, np.reshape(g_config_i, (1, link_num)))
                collision_cost_i = computeCollisionCost(task_pos_i, body_sizes, obstacles, radii, eps)

                J = robKine.computeJacobianConfig(g_config_i.flatten() * 180 / np.pi)
                vector_null = null_space(J)
                g_config_noise = g_config_i + 0.05 * np.reshape(vector_null, (1, link_num))

                task_pos_noise = traj_q2t(robKine, np.reshape(g_config_noise, (1, link_num)))
                collision_cost_noise = computeCollisionCost(task_pos_noise, body_sizes, obstacles, radii, eps)

                if np.abs(collision_cost_i - collision_cost_noise) < 1e-8:
                    break

                null_update_traj = np.zeros((T, link_num))
                null_update_traj[T - 1, :] = 0.05 * np.reshape(vector_null, (1, link_num)) * np.sign(collision_cost_i - collision_cost_noise)
                null_update_prop = np.dot(pinv_A, null_update_traj)
                traj_q = traj_q + null_update_prop

        while goal_error_norm > 0.003 or l < 50:
            x, J = robKine.computeJacobianEndPosConfig(traj_q[T - 1, :] * 180 / np.pi)
            goal_now = np.zeros((1, 3))
            goal_now[:] = x
            goal_error = goal_task - goal_now
            goal_error_norm = np.linalg.norm(goal_error)
            if goal_error_norm < 0.003 or l > 50:
                # print('ok', 'l', l)
                l = 0
                print('pos error ', goal_error_norm)
                break
            g_shift = np.zeros((6, 1))
            g_shift[0:3, 0] = goal_error[:]
            g_shift_q = robKine.ComputeJointVelfromCartVel(g_shift)
            # g_shift_q = np.linalg.pinv(J) * g_shift
            g_shift_traj = np.zeros((T, link_num))
            g_shift_traj[T - 1, :] = g_shift_q.flatten()
            g_shift_prop = np.dot(pinv_A, g_shift_traj)
            traj_q = traj_q + update_rate * g_shift_prop

            l += 1
            # if l == 50:
            #     print('pos error ', goal_error_norm)
            # print('l', l)

        traj_limit_low = np.tile(jointLimit_low, (T, link_num))
        traj_limit_high = np.tile(jointLimit_high, (T, link_num))

        # # Check the joint limit
        # if np.greater(traj_limit_low, traj_q).any():
        #     print('joint limit low')
        #     print('traj_q', traj_q)
        #     limit_shift = traj_limit_low - traj_q
        #     limit_shift_prop = np.dot(pinv_A, limit_shift)
        #     traj_q = traj_q + 0.01*limit_shift_prop
        #
        # if np.greater(traj_q, traj_limit_high).any():
        #     print('joint limit high')
        #     limit_shift = traj_limit_high - traj_q
        #     limit_shift_prop = np.dot(pinv_A, limit_shift)
        #     traj_q = traj_q + 0.01*limit_shift_prop

        # if np.abs(collision_cost - collision_cost_prev) < 1e-3:
        #     print('CHOMP converged.')
        #     break

        collision_cost_prev = collision_cost

        if collision_cost < collision_threshold:
            print('Collision cost smaller than threshold.')
            break

        if cnt > MaxIter:
            print('Reached the max number of iteration.')
            break


    print('collision_cost: %f, update num: %d \n' % (collision_cost, cnt))


    return traj_q







