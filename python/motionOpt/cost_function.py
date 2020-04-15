import numpy as np


def computeCost(robKine, traj_q, body_sizes, obstacles, radii, eps, cost_weights):
    traj_task = traj_q2t(robKine, traj_q)

    collision_cost = computeCollisionCost(traj_task, body_sizes, obstacles, radii, eps)
    smoothness_cost = computeSmoothnessCost(traj_q)

    cost_weights = cost_weights.squeeze()
    cost = cost_weights[0] * collision_cost + cost_weights[1] * smoothness_cost

    return cost, collision_cost, smoothness_cost


def computeCollisionCost(traj_task, body_sizes, obstacles, radii, eps):
    cost = []
    for t, traj_t in enumerate(traj_task):
        cost_row = []
        for b, body_point in enumerate(traj_t):
            cost_point = computeCollisionCostPoint(body_point[None, :], body_sizes.squeeze()[b], obstacles, radii, eps)
            cost_row.append(np.float_(cost_point))
        cost_row = np.stack(cost_row)
        cost.append(cost_row)

    cost = np.stack(cost)
    cost = cost.sum()

    return cost
     # cost_traj[t, b] = computeCollisionCostPoint(body_point, body_sizes[b], obstacles, radii, eps)


def computeCollisionCostPoint(x, bodySize, obstacles, radii, eps):
    n = radii.shape[0]
    diff = obstacles.transpose() - x.transpose()
    distance = np.sqrt(np.sum(np.square(diff), 0))
    distance = np.reshape(distance, (n, 1)) - radii - bodySize
    distance = np.vstack((distance, x[0, 2]))

    disMin = np.min(distance)

    if disMin < 0:
        cost = -disMin + eps * 0.5
    elif disMin < eps:
        cost = 0.5 * (disMin - eps) ** 2 / eps
    else:
        cost = 0

    return cost

def computeCostGradientPoint(x, bodySize, obstacles, radii, eps):
    # x needs to be of the shape 1xN
    n = x.shape
    cost_now = computeCollisionCostPoint(x, bodySize , obstacles, radii, eps)

    perturb_eps = 0.001

    grad_cost = np.zeros(n)

    for i in range(x.shape[1]):
        x_dx = np.copy(x)
        x_dx[0, i] = x_dx[0][i] + perturb_eps
        cost_dc = computeCollisionCostPoint(x_dx, bodySize, obstacles, radii, eps)
        grad_cost[0, i] = (cost_dc - cost_now)/perturb_eps

    return grad_cost

def computeSmoothnessCost(traj_q):
    T = traj_q.shape[0]
    K = np.diag([1] * T, 0) + np.diag([-1] * (T-1), -1)
    K[-1, -1] = 0

    A = K.T @ K

    vel_q = K @ traj_q
    accel_q = A @ traj_q

    cost = np.square(vel_q) + np.square(accel_q)
    cost = cost.sum()

    return cost

def traj_q2t(robKine, traj_q_rad):
    traj_list = []
    # jointsAngle in degree
    for row in traj_q_rad:
        x, _ = robKine.computeLinkPosJacob(row * 180 / np.pi)
        # x, _ = jacobian(robKine, row * 180 / np.pi)
        traj_list.append(x)

    traj_task = np.stack(traj_list, axis=0)

    return traj_task

# def jacobian(robKine, jointsAngle):
#     # jointsAngle in degree
#     jointNum = robKine.jointNum
#     robKine.setJoints(jointsAngle)
#     robKine.ForwardKinematics()
#     robKine.computeJacobian()
#
#     J_base = robKine.jacobian[:3]
#     rotMat = robKine.Hmat_worldbase2robBase[0:3,0:3]
#     J = rotMat * J_base
#     x = []
#     for i in range(robKine.jointNum):
#         H_w = np.dot(robKine.Hmat_worldbase2robBase, robKine.Hmat[i])
#         x.append(np.array(H_w[:3, 3]))
#     # x = [np.array(x[:3, 3]) for x in robKine.Hmat]
#     x = np.reshape(x, (jointNum, 3))
#     x_mid = (x[jointNum-2] + x[jointNum-1])*0.5
#     # print('x', x)
#     x = np.insert(x, jointNum-1, x_mid, axis=0)
#     # print('x after', x)
#     # input('press keyboard...')
#
#     return x, J