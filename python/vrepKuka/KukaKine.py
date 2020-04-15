import numpy as np
import numpy.matlib
import scipy as sp
from scipy.linalg import block_diag

import math
from pyquaternion import Quaternion

class KukaKine:
    def __init__(self, floor=False):
        # DH parameter for UR3
        self.DHParam =np.matrix([ [ 0.0, 90.0, 0.251, 0.0],
                        [0., -90.0, 0.0, 0.0],
                        [ 0.0, 90.0, 0.4, 0.0],
                        [0.0, -90.0, 0.0, 0.0],
                        [0.0, 90.0, 0.39, 0.0],
                        [0.0, -90.0, 0.0, 0.0],
                        # [0.0, 0.0, 0.355, 0.0]])
                        [-0.06, 0.0, 0.355, 0.0]])  # assuming the pregrasp position for the bottle, bag and bag2
                        # [-0.01, 0.0, 0.255, 0.0]])  # middle finger tip
                        # [-0.00, 0.0, 0.0675, 0.0]])  # link_8

        self.Weight_jacob =np.array([100, 10, 10, 10, 1, 1, 1] )
        self.Regularizer_C = 0.00001

        self.jacobian = np.matlib.zeros((6,7))
        self.joints = np.zeros((7,1))
        self.Hmat = [np.zeros((4,4))]*7
        self.endeffQuat = np.zeros((4,1))
        self.endeffPos = np.zeros((3,1))
        self.jointNum = 7
        self.H_joints = [np.matlib.zeros((4,4))]*7
        self.jointsAngle = np.zeros((7,))

        self.Hmat_worldbase2robBase = np.zeros((4, 4))
        self.Hmat_workdbase2robTip = np.zeros((4, 4))

        if floor==True:
            ## Darias on the floor at the origin
            self.Hmat_worldbase2robBase = np.identity(4)
            self.Hmat_worldbase2robBase[2,3] = 0.0595
        else:
            # ## Darias at TU Darmstadt setup
            Hmat1 = self.createHmatRotX(-90)
            Hmat2 = self.createHmatRotY(-30)
            Hmat3 = self.createHmatRotZ(90)
            HmatI = np.identity(4)

            HmatI[1, 3] = -0.0595
            HmatI[2, 3] = 0.72
            H_temp = np.dot(HmatI,Hmat1)
            H_temp2 = np.dot(H_temp, Hmat2)
            self.Hmat_worldbase2robBase =  np.dot(H_temp2, Hmat3)

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

        self.Hmat_workdbase2DariasTip = np.dot(self.Hmat_worldbase2robBase, self.Hmat[self.jointNum-1])
        self.computeEndEffQuat()
        self.endeffPos[:] = self.Hmat_workdbase2DariasTip[0:3, 3]
        # print('self.Hmat[1][0:3, 3]', self.Hmat[1][0:3, 3])

    def computeJacobian(self):
        self.jacobian[3, 0] = 0
        self.jacobian[4, 0] = 0
        self.jacobian[5, 0] = 1

        for i in range (1, self.jointNum):
            for j in range(3):
                self.jacobian[j+3, i] = self.Hmat[i-1][j, 2]

        vectorLink2End = np.matlib.zeros((1, 3))
        vectorLinkAxis = np.matlib.zeros((1, 3))

        for i in range(self.jointNum):
            if i == 0:
                for j in range(3):
                    vectorLink2End[0, j] = self.Hmat[self.jointNum-1][j, 3] - self.Hmat[0][j, 3]
                    vectorLinkAxis[0, j] = self.jacobian[j + 3, i]

            else:
                for j in range(3):
                    vectorLink2End[0, j] = self.Hmat[self.jointNum-1][j, 3] - self.Hmat[i-1][j, 3]
                    vectorLinkAxis[0, j] = self.jacobian[j + 3, i]

            Jv_i = np.cross(vectorLinkAxis, vectorLink2End)
            #print(vectorLinkAxis)
            #print(vectorLink2End)
            #print(Jv_i)

            for j in range(3):
                self.jacobian[j, i] = Jv_i[0, j]

    # def computeEndEffQuat(self):
    #     quat = Quaternion(matrix=self.Hmat_workdbase2DariasTip)
    #     self.endeffQuat = quat.elements

    def computeEndEffQuat(self):
        rotMat = self.Hmat_workdbase2DariasTip[0:3,0:3]
        self.endeffQuat = self.rotMat2Quat(rotMat)

    def rotMat2Quat(obj, RotMat):
        quat = np.zeros(4)
        quat[0] = np.sqrt(RotMat[0,0] +RotMat[1,1] + RotMat[2,2] +1)/2
        quat[1] = 0.5 * (RotMat[2,1] - RotMat[1,2]) / np.abs(RotMat[2,1] - RotMat[1,2])*np.sqrt( RotMat[0,0] - RotMat[1,1] - RotMat[2,2] +1 )
        quat[2] = 0.5 * (RotMat[0,2] - RotMat[2,0]) / np.abs(RotMat[0,2] - RotMat[2,0])* np.sqrt( - RotMat[0,0] + RotMat[1,1] - RotMat[2,2] +1)
        quat[3] = 0.5 * (RotMat[1,0] - RotMat[0,1]) / np.abs(RotMat[1,0] - RotMat[0,1])* np.sqrt( - RotMat[0,0] - RotMat[1,1] + RotMat[2,2] +1)
        return quat

    def computeQuatError(self, quatNow_array, quatDes_array):
        quatNow = Quaternion(numpy.array(quatNow_array))
        quatDes = Quaternion(numpy.array(quatDes_array))
        quatDiff = quatDes * quatNow.conjugate
        quatError = quatDiff.normalised

        return quatError.elements

    def quatError2AngluerVel(self, quatError_array):
        quatError = Quaternion(numpy.array(quatError_array))
        # print('quatError.norm', quatError.norm)
        if quatError.norm>1e-6:
            # print('quatError.norm', quatError.norm)
            theta = quatError.angle
            angular_vel = theta * quatError.elements[1:4] / quatError.norm
        else:
            angular_vel = np.array([0,0,0])
        return angular_vel

    def ComputeJointVelfromCartVel(self, vel_cart):
        # Rot_worldbase2DariasBase = obj.Hmat_worldbase2DariasBase(1:3, 1: 3)
        # R = blkdiag(Rot_worldbase2DariasBase',Rot_worldbase2DariasBase')
        # vel_dariasBase = R * vel_cart'
        # JWJT_C = obj.jacobianDarias * inv(W) * obj.jacobianDarias' + obj.Regularizer_C * eye(6,6)
        # vel_joint = transpose(inv(W) * obj.jacobianDarias' * inv(JWJT_C) * vel_dariasBase);

        Rot_world2rob = self.Hmat_worldbase2robBase[0:3,0:3]
        R = block_diag(np.transpose(Rot_world2rob), np.transpose(Rot_world2rob))
        # print('R', R)
        # print('vel_cart', vel_cart)
        vel_robBase = np.dot(R, np.reshape(vel_cart, (6,1)))
        # print('vel_robBase', vel_robBase)
        W_diag = np.diag(self.Weight_jacob)
        JWJT_C = self.jacobian * np.linalg.inv(W_diag)* np.transpose(self.jacobian) + self.Regularizer_C * np.identity(6)
        # print('JWJT_C', JWJT_C)
        vel_joint = np.linalg.inv(W_diag)* np.transpose(self.jacobian) * np.linalg.inv(JWJT_C) * vel_robBase

        # vel_joint = np.linalg.pinv(self.jacobian) * vel_cart

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
        # x = np.reshape(x, (self.jointNum, 3))
        # # print('jointsAngle', jointsAngle)
        # # print('x', x)

        J_base = self.jacobian[:3]
        rotMat = self.Hmat_worldbase2robBase[0:3, 0:3]
        J = rotMat * J_base
        x = []
        for i in range(self.jointNum):
            H_w = np.dot(self.Hmat_worldbase2robBase, self.Hmat[i])
            x.append(np.array(H_w[:3, 3]))

        # x = [np.array(x[:3, 3]) for x in self.Hmat]
        x = np.reshape(x, (self.jointNum, 3))
        x_mid = (x[self.jointNum - 2] + x[self.jointNum - 1]) * 0.5
        # print('x', x)
        x = np.insert(x, self.jointNum - 1, x_mid, axis=0)

        x[2, :] = 0.5 * x[1,:] + 0.5*x[3, :]
        x[4, :] = 0.5 * x[3, :] + 0.5 * x[5, :]

        return x, J

    def createHmatRotX(obj, rotAngle):
        angle_rad = rotAngle / 180 * np.pi
        HmatRotX = np.zeros((4, 4))
        HmatRotX[0, 0] = 1
        HmatRotX[1, 1] = np.cos(angle_rad)
        HmatRotX[1, 2] = np.sin(angle_rad)
        HmatRotX[2, 1] = -np.sin(angle_rad)
        HmatRotX[2, 2] = np.cos(angle_rad)
        HmatRotX[3, 3] = 1
        return HmatRotX

    def createHmatRotY(obj, rotAngle):
        angle_rad = rotAngle / 180 * np.pi
        HmatRotY = np.zeros((4, 4))
        HmatRotY[0, 0] = np.cos(angle_rad)
        HmatRotY[0, 2] = -np.sin(angle_rad)
        HmatRotY[1, 1] = 1
        HmatRotY[2, 0] = np.sin(angle_rad)
        HmatRotY[2, 2] = np.cos(angle_rad)
        HmatRotY[3, 3] = 1
        return HmatRotY

    def createHmatRotZ(obj, rotAngle):
        angle_rad = rotAngle / 180 * np.pi
        HmatRotZ = np.zeros((4, 4))
        HmatRotZ[0, 0] = np.cos(angle_rad)
        HmatRotZ[0, 1] = np.sin(angle_rad)
        HmatRotZ[1, 0] = -np.sin(angle_rad)
        HmatRotZ[1, 1] = np.cos(angle_rad)
        HmatRotZ[2, 2] = 1
        HmatRotZ[3, 3] = 1
        return HmatRotZ

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

