import numpy as np
import numpy.matlib
import numpy.linalg
import math
from pyquaternion import Quaternion

class UR3Kine:
    def __init__(self):
        # DH parameter for UR3
        self.DHParam =np.matrix([ [ 0.0,		90.0, 	0.1519, 0.0],
                        [-0.24365, 	0.0, 	0.11235,	0.0],
                        [-0.21325,		0.0, 	-0.11235, 0.0],
                        [0.,		90.0,	0.11235,	0.0],
                        [0.0,		-90.0,	0.08535,	0.0],
                        # [0.0,		0.0,	0.0819,	0.0]]) # no tool
                        [0.0, 0.0, 0.2819, 0.0]])

        # DH parameter for UR3e (e-series)
        # self.DHParam =np.matrix([ [ 0.0,		90.0, 	0.1519, 0.0],
        #                 [0.24355, 	0.0, 	0.13105,	90.0],
        #                 [0.2132,		0.0, 	-0.13105, 0.0],
        #                 [0.,		90.0,	0.13105,	0.0],
        #                 [0.0,		-90.0,	0.08535,	0.0],
        #                 [0.0,		0.0,	0.0921,	0.0] ])


        self.Weight_jacob =np.array([100, 100, 100, 10, 1, 1, 1] )
        self.Regularizer_C = 0.0001

        self.jacobian = np.matlib.zeros((6,6))
        self.joints = np.zeros((6,1))
        self.Hmat = [np.zeros((4,4))]*6
        self.endeffQuat = np.zeros((4,1))
        self.endeffPos = np.zeros((3,1))
        self.jointNum = 6
        self.H_joints = [np.matlib.zeros((4,4))]*6
        self.jointsAngle = np.zeros((6,)) #np.array([0,0,0,0,0,0])
        self.Hmat_worldbase2robBase = np.identity(4)

    def setJoints(self, jointsAngle_set):
        # assert self.jointsAngle.shape == jointsAngle_set.shape
        self.jointsAngle[:] = jointsAngle_set[:]

    def convertDH2Hmat(self, jointidx):
        Hmat = np.matlib.zeros((4, 4))

        a = self.DHParam[jointidx, 0]
        alpha = self.DHParam[jointidx, 1] * math.pi / 180
        d = self.DHParam[jointidx, 2]
        theta = self.jointsAngle[jointidx] * math.pi / 180 + self.DHParam[jointidx, 3]* math.pi / 180

        Hmat[0, 0] = np.cos(theta)
        Hmat[0, 1] = -np.sin(theta) * np.cos(alpha)
        Hmat[0, 2] = np.sin(theta) * np.sin(alpha)
        Hmat[0, 3] = a * np.cos(theta)
        Hmat[1, 0] = np.sin(theta)
        Hmat[1, 1] = np.cos(theta) * np.cos(alpha)
        Hmat[1, 2] = -np.cos(theta) * np.sin(alpha)
        Hmat[1, 3] = a * np.sin(theta)
        Hmat[2, 1] = np.sin(alpha)
        Hmat[2, 2] = np.cos(alpha)
        Hmat[2, 3] = d
        Hmat[3, 3] = 1

        return Hmat

    def computeHmatJoint(self):
        for i in range(self.jointNum):
            self.H_joints[i] = self.convertDH2Hmat(i)

    def ForwardKinematics(self):
        self.computeHmatJoint()

        self.Hmat[0] = self.H_joints[0]

        for i in range(1, self.jointNum):
            self.Hmat[i] = self.Hmat[i - 1] *self.H_joints[i]

        #obj.Hmat_workdbase2DariasTip = obj.Hmat_worldbase2DariasBase * obj.DariasHmat(:,:, 7)
        self.computeEndEffQuat()
        self.endeffPos[:] = self.Hmat[5][0:3, 3]

        # print('self.Hmat[1][0:3, 3]', self.Hmat[1][0:3, 3])


    def computeJacobian(self):
        self.jacobian[3, 0] = 0
        self.jacobian[4, 0] = 0
        self.jacobian[5, 0] = 1

        for i in range (1, self.jointNum):
            for j in range (3):
                self.jacobian[j+3, i] = self.Hmat[i-1][j, 2]

        vectorLink2End = np.matlib.zeros((1, 3))
        vectorLinkAxis = np.matlib.zeros((1, 3))

        for i in range(self.jointNum):
            if i == 0:
                for j in range(3):
                    vectorLink2End[0, j] = self.Hmat[5][j, 3] - self.Hmat[0][j, 3]
                    vectorLinkAxis[0, j] = self.jacobian[j + 3, i]

            else:
                for j in range(3):
                    vectorLink2End[0, j] = self.Hmat[5][j, 3] - self.Hmat[i-1][j, 3]
                    vectorLinkAxis[0, j] = self.jacobian[j + 3, i]

            Jv_i = np.cross(vectorLinkAxis, vectorLink2End)
            #print(vectorLinkAxis)
            #print(vectorLink2End)
            #print(Jv_i)

            for j in range(3):
                self.jacobian[j, i] = Jv_i[0, j]

    def computeEndEffQuat(self):
        quat = Quaternion(matrix=self.Hmat[5])
        self.endeffQuat = quat.elements

    def computeQuatError(self, quatNow_array, quatDes_array):
        quatNow = Quaternion(numpy.array(quatNow_array))
        quatDes = Quaternion(numpy.array(quatDes_array))
        quatDiff = quatDes * quatNow.conjugate
        quatError = quatDiff.normalised

        return quatError.elements

    def quatError2AngluerVel(self, quatError_array):
        quatError = Quaternion(numpy.array(quatError_array))
        theta = quatError.angle
        angular_vel = theta * quatError.elements[1:4] / quatError.norm
        return angular_vel

    def ComputeJointVelfromCartVel(self, vel_cart):
        vel_joint = np.linalg.pinv(self.jacobian) * vel_cart
        return vel_joint

    def normalizeQuat(self, quat_array):
        quat = Quaternion(numpy.array(quat_array))
        return quat.normalised.elements


    def computeJacobianConfig(self, config_deg):
        self.setJoints(config_deg)
        self.ForwardKinematics()
        self.computeJacobian()

        return self.jacobian

    def computeEndPosConfig(self, config_deg):
        # config in degree
        self.setJoints(config_deg)
        self.ForwardKinematics()

        return self.endeffPos.reshape((1,3))

    def computeJacobianEndPosConfig(self, config_deg):
        # config in degree
        self.setJoints(config_deg)
        self.ForwardKinematics()
        self.computeJacobian()

        return self.endeffPos.reshape((1,3)), self.jacobian

    def convertConfigTraj2TaskTraj(self, traj_q):
        T = traj_q.shape[0]
        traj_task = np.zeros((T, 3))
        for t in range(T):
            self.setJoints(traj_q[t, :]*180/np.pi)
            self.ForwardKinematics()
            traj_task[t, :] = self.endeffPos.flatten()

        return traj_task

    def computeLinkPosJacob(self, jointsAngle):
        # jointsAngle in degree
        self.setJoints(jointsAngle)
        self.ForwardKinematics()
        self.computeJacobian()

        # J = self.jacobian[:3]
        # x = [np.array(x[:3, 3]) for x in self.Hmat]
        # x = np.reshape(x, (6, 3))
        # # print('jointsAngle', jointsAngle)
        # # print('x', x)

        J_base = self.jacobian[:3]
        rotMat = self.Hmat_worldbase2robBase[0:3, 0:3]
        J = rotMat * J_base
        x = [np.array(x[:3, 3]) for x in self.Hmat]
        x = np.reshape(x, (self.jointNum, 3))
        x_mid = (x[self.jointNum - 2] + x[self.jointNum - 1]) * 0.5
        # print('x', x)
        x = np.insert(x, self.jointNum - 1, x_mid, axis=0)

        return x, J




    # def rotMat2Quat(self, RotMat):
    #     quat = np.matlib.zeros((1, 4))
    #     quat[0,0] = np.sqrt(RotMat[0, 0] + RotMat[1, 1] + RotMat[2, 2] + 1) / 2.0
    #
    #     quat[0,1] = 0.5 * (RotMat[2, 1] - RotMat[1, 2]) / np.abs(RotMat[2, 1] - RotMat[1, 2]) * np.sqrt(RotMat[0, 0] - RotMat[1, 1] - RotMat[2, 2] + 1)
    #
    #     quat[0,2] = 0.5 * (RotMat[0, 2] - RotMat[2, 0]) / np.abs(RotMat[0, 2] - RotMat[2, 0]) *np.sqrt(- RotMat[0, 0] + RotMat[1, 1] - RotMat[2, 2] + 1)
    #
    #     quat[0,3] = 0.5 * (RotMat[1, 0] - RotMat[0, 1]) / np.abs(RotMat[1, 0] - RotMat[0, 1]) *np.sqrt(- RotMat[0, 0] - RotMat[1, 1] + RotMat[2, 2] + 1)
    #
    #     return quat



