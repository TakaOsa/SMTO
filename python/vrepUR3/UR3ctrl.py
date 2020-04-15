try:
    import vrepPyApi.vrep as vrep
except:
    print ('--------------------------------------------------------------')
    print ('"vrep.py" could not be imported. This means very probably that')
    print ('either "vrep.py" or the remoteApi library could not be found.')
    print ('Make sure both are in the same folder as this file,')
    print ('or appropriately adjust the file "vrep.py"')
    print ('--------------------------------------------------------------')
    print ('')

import numpy as np
import numpy.matlib
import time
import math


class UR3:
    def __init__(self):
        self.ClientID = -1
        self.jointNum = 6
        self.jointHandles = np.zeros(6)
        self.jointStates = np.zeros(6)

    def startCommunication(self):
        self.clientID = vrep.simxStart('127.0.0.1', 19997, True, True, 5000, 5)
        if self.clientID != -1:
            print('Connected to remote API server')

    def stopCommunication(self):
        # Before closing the connection to V-REP, make sure that the last command sent out had time to arrive. You can guarantee this with (for example):
        vrep.simxGetPingTime(self.clientID)
        vrep.simxFinish(self.clientID)
        print('Disconnected to remote API server')

    def startSimulation(self):
        vrep.simxStartSimulation(self.clientID, vrep.simx_opmode_oneshot_wait)
        print('Start simulation.')

    def stopSimulation(self):
        vrep.simxStopSimulation(self.clientID, vrep.simx_opmode_oneshot_wait)
        print('Stop simulation')

    def pauseSimulation(self):
        vrep.simxPauseSimulation(self.clientID, vrep.simx_opmode_oneshot_wait)
        print('Simulation is paused')

    def getArmHandles(self):
        self.jointHandles[0] = self.getHandle('UR3_joint1')
        self.jointHandles[1] = self.getHandle('UR3_joint2')
        self.jointHandles[2] = self.getHandle('UR3_joint3')
        self.jointHandles[3] = self.getHandle('UR3_joint4')
        self.jointHandles[4] = self.getHandle('UR3_joint5')
        self.jointHandles[5] = self.getHandle('UR3_joint6')

    def getHandle(self, objName):
        res, handle = vrep.simxGetObjectHandle(self.clientID, objName, vrep.simx_opmode_blocking)
        if res == vrep.simx_return_ok:
            print(objName, handle)
            return handle
        else:
            return -1

    def getArmConfig(self):
        for i in range(self.jointNum):
            self.jointStates[i] = self.getJointPos(i)

        self.jointStates[1] = self.jointStates[1] - np.pi*0.5
        self.jointStates[3] = self.jointStates[3] - np.pi*0.5
        self.jointStates[5] = self.jointStates[5] + np.pi*0.5

        return self.jointStates

    def getJointPos(self, joint_idx):
        res, joint_rad = vrep.simxGetJointPosition(self.clientID, int(self.jointHandles[joint_idx]),
                                                   vrep.simx_opmode_oneshot_wait )
        # print(joint_idx, joint_rad)
        return joint_rad # The unit of the value from vrep is radian, not degree


    def setTargetJointPos(self, joint_target, joint_idx):

        res = vrep.simxSetJointTargetPosition(self.clientID, int(self.jointHandles[joint_idx]),
                                              joint_target, vrep.simx_opmode_oneshot )

    def setJointPos(self, joint_set, joint_idx):
        ret = vrep.simxSetJointPosition (self.clientID, int(self.jointHandles[joint_idx]),
                                         joint_set, vrep.simx_opmode_oneshot )

    def setTargetArmConfig(self, jointsArm_target):
        jointsArm_target_vrep = np.copy(jointsArm_target)
        jointsArm_target_vrep[1] += np.pi*0.5
        jointsArm_target_vrep[3] += np.pi*0.5
        jointsArm_target_vrep[5] -= np.pi*0.5

        for i in range(6):
            self.setTargetJointPos(jointsArm_target_vrep[i], i)

    def setArmPos(self, jointsArm_set):
        joint_set_vrep = np.copy(jointsArm_set)
        joint_set_vrep[1] += np.pi*0.5
        joint_set_vrep[3] += np.pi*0.5
        joint_set_vrep[5] -= np.pi*0.5

        for i in range(6):
            self.setJointPos(joint_set_vrep[i], i)

    def sendTargetTrajectoryConfig(self, traj_config, rate):
        n = traj_config.shape[0]
        for i in range(n):
            self.setTargetArmConfig(traj_config[i])
            time.sleep(1/rate)

    def setTargetTrajectoryArmTaskSpace(self, traj_taskspace, robKine, rate):
        n = traj_taskspace.shape[0]
        trajConfig = np.zeros((n, self.jointNum))
        jointArm = self.getArmConfig()
        print('jointArm_ini', jointArm)
        robKine.setJoints(jointArm*180.0/math.pi)
        robKine.ForwardKinematics()
        pos_ini = robKine.endeffPos
        quat_ini = robKine.endeffQuat
        quatDesired_ini = traj_taskspace[0, 3:7]
        trajConfig[0] = jointArm
        vel_cart = np.matlib.zeros((1,6))
        print('pos_ini', pos_ini)

        for i in range(n-1):
            for j in range(5):
                robKine.setJoints(jointArm*180.0/ math.pi)
                robKine.ForwardKinematics()
                robKine.computeJacobian()

                pos = robKine.endeffPos

                vel_cart_trans = traj_taskspace[i + 1, 0:3] - traj_taskspace[i, 0: 3]

                quatDesired_now = traj_taskspace[i, 3:7]
                quat_now = robKine.endeffQuat

                quatError = robKine.computeQuatError(traj_taskspace[i, 3:7], traj_taskspace[i+1, 3:7])
                quatMoved = robKine.computeQuatError(quat_ini, quat_now)
                quatMoved_desired = robKine.computeQuatError(quatDesired_ini, quatDesired_now)
                quatMoved_error = robKine.computeQuatError(quatMoved, quatMoved_desired)

                vel_angular = robKine.quatError2AngluerVel(quatError)

                if j > 0:
                    vel_cart_trans = np.zeros((3,))
                    vel_angular = np.zeros((3,))
                    # error_trans = np.asarray(pos - pos_ini).reshape((3,)) - (traj_taskspace[i, 0:3] - traj_taskspace[0, 0:3] )
                    error_trans = np.asarray(pos).reshape((3,)) - traj_taskspace[i, 0:3]
                    vel_angular_error = robKine.quatError2AngluerVel(quatMoved_error)
                else:
                    error_trans = np.zeros((3,))
                    vel_angular_error = np.zeros((3,))


                vel_trans = vel_cart_trans - 0.01 * error_trans
                vel_ang = vel_angular + 0.5 * vel_angular_error

                # print(vel_ang)
                # print(vel_cart_trans)
                vel_cart[0, 0:3] = vel_trans[0:3]
                vel_cart[0, 3:6] = vel_ang[0:3]

                vel_joint = robKine.ComputeJointVelfromCartVel(vel_cart.transpose())
                jointArm = np.asarray(vel_joint).reshape((1, 6)) + jointArm
                jointArm = np.asarray(jointArm).reshape(6)

            trajConfig[i + 1, 0:6] = jointArm
            # print(jointArm)
            self.setTargetArmConfig(jointArm)
            time.sleep(1/rate)

        print('robKine.endeffPos',robKine.endeffPos)

        return trajConfig

    def point2pointMotion(self, robKine, goalPos, goalQuat, stepnum, freq):
        # jointArm = self.getArmConfig()
        # robKine.setJoints(jointArm*180/np.pi)
        print('robKine.joints', robKine.joints)
        robKine.ForwardKinematics()
        pos_ini = robKine.endeffPos
        quat_ini = robKine.endeffQuat
        traj = np.zeros((stepnum, 7))

        for i in range(stepnum):
            pos_i = pos_ini + (goalPos - pos_ini) * (1 - np.cos((i - 1) / (stepnum - 1) * math.pi)) / 2
            traj[i, 0: 3] = pos_i.reshape(3)
            quat_i = quat_ini + (goalQuat - quat_ini) * (1 - np.cos((i - 1) / (stepnum - 1) * math.pi)) / 2
            quat_i_normed = robKine.normalizeQuat(quat_i)
            traj[i, 3: 7] = quat_i_normed

        # print('traj',traj)

        trajConfig = self.setTargetTrajectoryArmTaskSpace(traj, robKine, freq)

        return trajConfig

    def sendTestMssg(self):
        vrep.simxAddStatusbarMessage(self.clientID, 'Hello V-REP!', vrep.simx_opmode_oneshot)
