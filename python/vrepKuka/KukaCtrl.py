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

class KukaCtrl:
    def __init__(self, Barret=False):
        self.ClientID = -1
        self.jointNum = 7
        self.jointHandles = np.zeros(7)
        self.jointStates = np.zeros(7)
        self.fingerJointHandle = np.zeros((5,4))
        self.fingerJointStates = np.zeros((5,4))

        self.fingerNum =5
        self.fingerJointNum =4
        self.THUMB = 0
        self.INDEX = 1
        self.MID = 2
        self.RING = 3
        self.SMALL = 4

        self.SPREAD = 0
        self.PROXIMAL = 1
        self.DISTAL = 2
        self.TIP = 3

        if Barret:
            self.fingerNum = 2
            self.fingerJointHandle = np.zeros((2,2))
            self.fingerJointStates = np.zeros((2,2))

        self.Barret=Barret


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
        self.jointHandles[0] = self.getHandle('LBR4p_joint1')
        self.jointHandles[1] = self.getHandle('LBR4p_joint2')
        self.jointHandles[2] = self.getHandle('LBR4p_joint3')
        self.jointHandles[3] = self.getHandle('LBR4p_joint4')
        self.jointHandles[4] = self.getHandle('LBR4p_joint5')
        self.jointHandles[5] = self.getHandle('LBR4p_joint6')
        self.jointHandles[6] = self.getHandle('LBR4p_joint7')

    def getFingerHandles(self):
        if self.Barret:
            self.fingerJointHandle[0, 1] = self.getHandle('Barrett_openCloseJoint')
            self.fingerJointHandle[1, 1] = self.getHandle('Barrett_openCloseJoint0')
            self.fingerJointHandle[0, 0] = self.getHandle('BarrettHand_jointA_0')
            self.fingerJointHandle[1, 0] = self.getHandle('BarrettHand_jointA_2')
        else:
            self.fingerJointHandle[self.INDEX, self.SPREAD] = self.getHandle('right_hand_index_finger_spread_joint')
            self.fingerJointHandle[self.INDEX, self.PROXIMAL] = self.getHandle('right_hand_index_finger_proximal_joint')
            self.fingerJointHandle[self.INDEX, self.DISTAL] = self.getHandle('right_hand_index_finger_distal_joint')
            self.fingerJointHandle[self.INDEX, self.TIP] = self.getHandle('right_hand_index_finger_tip_joint')

            self.fingerJointHandle[self.MID, self.SPREAD] = self.getHandle('right_hand_middle_finger_spread_joint')
            self.fingerJointHandle[self.MID, self.PROXIMAL] = self.getHandle('right_hand_middle_finger_proximal_joint')
            self.fingerJointHandle[self.MID, self.DISTAL] = self.getHandle('right_hand_middle_finger_distal_joint')
            self.fingerJointHandle[self.MID, self.TIP] = self.getHandle('right_hand_middle_finger_tip_joint')

            self.fingerJointHandle[self.RING, self.SPREAD] = self.getHandle('right_hand_ring_finger_spread_joint')
            self.fingerJointHandle[self.RING, self.PROXIMAL] = self.getHandle('right_hand_ring_finger_proximal_joint')
            self.fingerJointHandle[self.RING, self.DISTAL] = self.getHandle('right_hand_ring_finger_distal_joint')
            self.fingerJointHandle[self.RING, self.TIP] = self.getHandle('right_hand_ring_finger_tip_joint')

            self.fingerJointHandle[self.SMALL, self.SPREAD] = self.getHandle('right_hand_small_finger_spread_joint')
            self.fingerJointHandle[self.SMALL, self.PROXIMAL] = self.getHandle('right_hand_small_finger_proximal_joint')
            self.fingerJointHandle[self.SMALL, self.DISTAL] = self.getHandle('right_hand_small_finger_distal_joint')
            self.fingerJointHandle[self.SMALL, self.TIP] = self.getHandle('right_hand_small_finger_tip_joint')

            self.fingerJointHandle[self.THUMB, self.SPREAD] = self.getHandle('right_hand_thumb_spread_joint')
            self.fingerJointHandle[self.THUMB, self.PROXIMAL] = self.getHandle('right_hand_thumb_proximal_joint')
            self.fingerJointHandle[self.THUMB, self.DISTAL] = self.getHandle('right_hand_thumb_distal_joint')
            self.fingerJointHandle[self.THUMB, self.TIP] = self.getHandle('right_hand_thumb_tip_joint')

    def getHandle(self, objName):
        res, handle = vrep.simxGetObjectHandle(self.clientID, objName, vrep.simx_opmode_blocking)
        if res == vrep.simx_return_ok:
            print(objName, handle)
            return handle
        else:
            return -1

    def setObjectPos(self, objHandle, pos):
        res = vrep.simxSetObjectPosition(self.clientID, int(objHandle), -1, pos, vrep.simx_opmode_oneshot)

    def getArmConfig(self):
        for i in range(self.jointNum):
            self.jointStates[i] = self.getJointPos(i)

        # self.jointStates[1] = self.jointStates[1] - np.pi * 0.5

        return self.jointStates

    def getJointPos(self, joint_idx):
        res, joint_rad = vrep.simxGetJointPosition(self.clientID, int(self.jointHandles[joint_idx]),
                                                   vrep.simx_opmode_oneshot_wait )

        return joint_rad # The unit of the value from vrep is radian, not degree


    def setTargetJointPos(self, joint_target, joint_idx):
        res = vrep.simxSetJointTargetPosition(self.clientID, int(self.jointHandles[joint_idx]),
                                              joint_target, vrep.simx_opmode_oneshot )
        # if not res == 0:
        #     print('communication error in setTargetJointPos')

    def setJointPos(self, joint_set, joint_idx):
        ret = vrep.simxSetJointPosition (self.clientID, int(self.jointHandles[joint_idx]),
                                         joint_set, vrep.simx_opmode_oneshot )

    def setTargetArmConfig(self, jointsArm_target):
        jointsArm_target_vrep = np.copy(jointsArm_target)
        # jointsArm_target_vrep[1] += np.pi * 0.5
        for i in range(self.jointNum):
            self.setTargetJointPos(jointsArm_target_vrep[i], i)


    def setArmPos(self, jointsArm_set):
        joint_set_vrep = np.copy(jointsArm_set)
        # joint_set_vrep[1] += np.pi * 0.5

        for i in range(self.jointNum):
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
        robKine.setJoints(jointArm*180.0/np.pi)
        robKine.ForwardKinematics()
        pos_ini = robKine.endeffPos
        quat_ini = robKine.endeffQuat
        quatDesired_ini = traj_taskspace[0, 3:7]
        trajConfig[0] = jointArm
        vel_cart = np.matlib.zeros((1,6))
        print('pos_ini', pos_ini)

        for i in range(n-1):
            for j in range(5):
                robKine.setJoints(jointArm*180.0/ np.pi)
                robKine.ForwardKinematics()
                robKine.computeJacobian()

                pos = robKine.endeffPos

                vel_cart_trans = traj_taskspace[i + 1, 0:3] - traj_taskspace[i, 0: 3]
                error_trans = np.asarray(pos).reshape((3,)) - traj_taskspace[i, 0:3]

                quatDesired_now = traj_taskspace[i, 3:7]
                quat_now = robKine.endeffQuat

                quatError = robKine.computeQuatError(traj_taskspace[i, 3:7], traj_taskspace[i+1, 3:7])
                quatMoved = robKine.computeQuatError(quat_ini, quat_now)
                quatMoved_desired = robKine.computeQuatError(quatDesired_ini, quatDesired_now)
                quatMoved_error = robKine.computeQuatError(quatMoved, quatMoved_desired)
                # quatMoved_error = robKine.computeQuatError(quat_now, quatDesired_now)

                vel_angular_error = robKine.quatError2AngluerVel(quatMoved_error)
                vel_angular = robKine.quatError2AngluerVel(quatError)

                if j > 0:
                    vel_cart_trans = np.zeros((3,))
                    vel_angular = np.zeros((3,))
                    # error_trans = np.asarray(pos - pos_ini).reshape((3,)) - (traj_taskspace[i, 0:3] - traj_taskspace[0, 0:3] )

                # else:
                    # error_trans = np.zeros((3,))
                    # vel_angular_error = np.zeros((3,))
                    # print('traj_taskspace[i, 3:7]', traj_taskspace[i, 3:7])

                # print('vel_angular', vel_angular)
                vel_trans = vel_cart_trans - 0.01 * error_trans
                vel_ang = vel_angular + 0.5 * vel_angular_error
                # print('vel_ang', vel_ang)

                # print(vel_ang)
                # print(vel_cart_trans)
                vel_cart[0, 0:3] = vel_trans[0:3]
                vel_cart[0, 3:6] = vel_ang[0:3]

                vel_joint = robKine.ComputeJointVelfromCartVel(vel_cart.transpose())
                # print('vel_joint', vel_joint)

                jointArm = np.asarray(vel_joint).reshape((1, self.jointNum)) + jointArm
                jointArm = np.asarray(jointArm).reshape(self.jointNum)

            trajConfig[i + 1, 0:self.jointNum] = jointArm
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

    def getFingerJointPos(self,  fingerNum, fingerJointNum):
        res, joint_rad = vrep.simxGetJointPosition(self.clientID, int(self.fingerJointHandle[fingerNum, fingerJointNum]),
                                                   vrep.simx_opmode_oneshot_wait)
        jointAngle = joint_rad * 180 /np.pi
        return jointAngle

    def getHandConfig(self):
        for i in range(self.fingerNum):
            for j in range(self.fingerJointNum):
                self.fingerJointStates[i,j] = self.getFingerJointPos(i, j)

        return self.fingerJointStates

    def setFingerJointPos(self, joint_set, fingerIdx, fingerJointIdx):
        ret = vrep.simxSetJointPosition (self.clientID, int(self.fingerJointHandle[fingerIdx, fingerJointIdx]),
                                         joint_set, vrep.simx_opmode_oneshot )

    def setTargetFingerJointPos(self, joint_set, fingerNum, fingerJointNum):
        ret = vrep.simxSetJointTargetPosition (self.clientID, int(self.fingerJointHandle[fingerNum, fingerJointNum]),
                                         joint_set, vrep.simx_opmode_oneshot )

    def setTargetHandConfigBarret(self, handConfig_target):
        for i in range(self.fingerNum):
            for j in range(2):
                finger_target_i = handConfig_target[i, j] * np.pi / 180
                self.setTargetFingerJointPos(finger_target_i, i, j)

    def setTargetFingerCoupled(self, joint_finger_set, finger_idx):
        joint_set_i = joint_finger_set / (self.fingerJointNum -1)
        for i in range(1, self.fingerJointNum):
            self.setTargetFingerJointPos(joint_set_i, finger_idx, i)

    def setTargetHandConfig(self, handConfig_target):
        for i in range(self.fingerNum):
            finger_target_i = handConfig_target[i] * np.pi /180
            self.setTargetFingerCoupled(finger_target_i, i)

    def sendTestMssg(self):
        vrep.simxAddStatusbarMessage(self.clientID, 'Hello V-REP!', vrep.simx_opmode_oneshot)
