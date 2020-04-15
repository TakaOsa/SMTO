import time
import numpy as np

from vrepKuka.KukaCtrl import KukaCtrl
from vrepKuka.KukaKine import KukaKine
from motionOpt.multiTrajOpt import multiTrajOpt

T = np.int_(50)
ite_num = 3
dim = np.int_(3)

# setting for obstacles
obsNum = np.int_(2)
obstacles = np.array([[0.63, -0.2, 0.3],[0.63, -0.2, 0.2]], ndmin=2)
radii = np.array([[0.1], [0.2]])
eps = np.float_(0.06)
costWeights = np.array([1.0, 0.01])

bodySizes = np.array([0.15, 0.15, 0.15, 0.1, 0.1, 0.1, 0.1, 0.25])
iteNum = np.int_(10)
collisionThreshold = np.float_(1.5)
updateRate = np.float_(0.04)

signal_rate = 25

print ('Program started')
robCon = KukaCtrl()
robKine = KukaKine()

ArmConfig_ini = np.array([-28.9806, - 49.8527, 128.8750, - 85.5503, 59.7840, 63.7785, -5.0670])

robCon.startCommunication()
robCon.getArmHandles()

time.sleep(1)
robCon.stopSimulation()
time.sleep(1)

jointsAngle = ArmConfig_ini * np.pi /180
jointsAngle_deg = ArmConfig_ini

robCon.setArmPos(jointsAngle) # important for p2p
robCon.setTargetArmConfig(jointsAngle) # important for p2p
robKine.setJoints(jointsAngle_deg)

robKine.ForwardKinematics()

robCon.startSimulation()
time.sleep(1)

GoalPos = np.array([0.68, 0.05, 0.23]).reshape((3, 1))
GoalQuat = np.array([-8.56595855e-02, 9.77097699e-01, -3.94075988e-04, 1.94788004e-01])

traj_ini_p2p_rad = robCon.point2pointMotion(robKine, GoalPos, GoalQuat, T, 25)

time.sleep(2)
robCon.stopSimulation()
time.sleep(2)

tic = time.time()
sampleNum = 800
MaxSol = 10
scale = 20
coll_thre = 1.7
seed_num = 2

trajConfig_mulTrajOpt, modeNum, cost_m = multiTrajOpt(robKine, traj_ini_p2p_rad, T, ite_num,
                                                      obstacles, radii, eps, bodySizes, costWeights,
                                                      sampleNum=sampleNum, collision_threshold= coll_thre,
                                                      MaxSol=MaxSol, scale=scale, seed_num=seed_num)

toc = time.time()
print( 'Elapsed time: ', (toc - tic), 'sec' )

input("Press Enter to continue...")

foldername = 'out/'
taskname = 'kuka_bag/kuka_bag_'

T_spline = 100
t  = np.linspace(0, 1, T)
ts = np.linspace(0, 1, T_spline)
traj_m_spline = np.zeros((T_spline, 7))
from scipy.interpolate import UnivariateSpline

for m in range(modeNum):
    print('mode ', m)

    trajTask_opt_m = robKine.convertConfigTraj2TaskTraj(trajConfig_mulTrajOpt[m])

    fname_opt_m = foldername + taskname + 'OptPath_mode' + str(m) + '.csv'
    np.savetxt(fname_opt_m, trajConfig_mulTrajOpt[m], delimiter=',')

    fname_opt_task_m = foldername + taskname + 'OptPath_task_mode' + str(m) + '.csv'
    np.savetxt(fname_opt_task_m, trajTask_opt_m, delimiter=',')

    for i in range(7):
        s = UnivariateSpline(t, trajConfig_mulTrajOpt[m, :, i] )
        traj_m_spline[:, i] = s(ts)

    time.sleep(0.1)
    robCon.stopSimulation()
    time.sleep(1)
    robCon.startSimulation()
    time.sleep(0.1)

    robCon.sendTargetTrajectoryConfig(traj_m_spline, signal_rate)
    time.sleep(1)

robCon.stopSimulation()
time.sleep(2)

robCon.stopCommunication()
print ('Program ended')