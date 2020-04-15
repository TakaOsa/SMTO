import time
import numpy as np

from vrepKuka.KukaCtrl import KukaCtrl
from vrepKuka.KukaKine import KukaKine

from motionOpt.multiTrajOptEnd import multiTrajOptEnd

T = np.int_(50)
ite_num = 3
dim = np.int_(3)
obsNum = np.int_(4)
obstacles = np.array([[0.0, -0.45, 0.4],[0., -0.45, 0.3],[0., -0.45, 0.2], [0.0, -0.7, 0.15]], ndmin=2)
radii = np.float_([[0.05], [0.05], [0.05], [0.05]])
bodySizes = np.array([0.15, 0.15, 0.15, 0.15, 0.15, 0.15, 0.1, 0.2])
eps = np.float_(0.1)
costWeights = np.array([1.0, 0.01])
iteNum = np.int_(10)
collisionThreshold = np.float_(1.5)
updateRate = np.float_(0.04)

signal_rate = 25

print ('Program started')
robCon = KukaCtrl()
robKine = KukaKine(floor=True)

jointsAngle_ini =np.array([50,-45,0,50,0, 45,0])* np.pi / 180
HandConfig_close = np.array([50, 90, 90, 90, 90])
HandConfig_open = np.array([0, 0, 0, 0, 0])

robCon.startCommunication()
robCon.getArmHandles()
robCon.getFingerHandles()
time.sleep(1)
robCon.stopSimulation()
time.sleep(1)


jointsAngle_ini_deg = jointsAngle_ini * 180 /np.pi

robCon.setArmPos(jointsAngle_ini) # important for p2p
robCon.setTargetArmConfig(jointsAngle_ini) # important for p2p
robKine.setJoints(jointsAngle_ini_deg)
robKine.ForwardKinematics()


jointsAngle_g = np.array([ 1.46949161, 0.78232318, -0.07518206, 2.18347126, -0.15733742, -1.40286821, 1.47672952])

traj_ini_p2p_rad = np.zeros((T, 7))
for t in range(T):
    traj_ini_p2p_rad[t, :] = 0.5*(np.cos(t/(T-1)*np.pi) + 1)* jointsAngle_ini + 0.5 * (1 - np.cos(t/(T-1)*np.pi))* jointsAngle_g
robCon.startSimulation()
time.sleep(1)
robCon.sendTargetTrajectoryConfig(traj_ini_p2p_rad, 25)

robKine.setJoints(jointsAngle_g.flatten() * 180/np.pi)
robKine.ForwardKinematics()
time.sleep(1)

print('robKine.endeffPos', robKine.endeffPos)
print('robKine.endeffQuat', robKine.endeffQuat)

robCon.stopSimulation()
time.sleep(2)

angle_pos =  np.pi*0.35
angle_neg = np.pi*0.35
freeAxis = np.array([0, 0, 1])
jointLimit_low = np.array([-np.pi*2, -np.pi*2, -np.pi*2, -np.pi*2, -np.pi*2, -np.pi*2, -np.pi*2])
jointLimit_high = np.array([ np.pi*2, np.pi*2, np.pi*2, np.pi*2, np.pi*2, np.pi*2, np.pi*4])

sampleNum = 800
MaxSol = 10
scale = 20
seed_num = 0

tic = time.time()

trajConfig_mulTrajOpt, modeNum, cost_m, D_latent, cost_set, modes = multiTrajOptEnd(robKine, traj_ini_p2p_rad, T, ite_num, obstacles, radii, eps, bodySizes, costWeights,
                                                 angle_pos, angle_neg, freeAxis, jointLimit_low, jointLimit_high,
                                                sampleNum=sampleNum, opt_threshold=3.0,MaxSol=MaxSol, scale=scale, seed_num=seed_num )

toc = time.time()

print( 'Elapsed time: ', (toc - tic), 'sec' )

foldername = 'out/'
taskname = 'kuka_bottle/kuka_bottle_'

fname_traj_latent = foldername + taskname + 'TrajSample_latent.csv'
np.savetxt(fname_traj_latent, D_latent, delimiter=',')

fname_cost = foldername + taskname + 'cost_set.csv'
np.savetxt(fname_cost, cost_set, delimiter=',')

fname_modes = foldername + taskname + 'modes.csv'
np.savetxt(fname_modes, modes, delimiter=',')

input("Press Enter to continue...")

T_spline = 200
t  = np.linspace(0, 1, T)
ts = np.linspace(0, 1, T_spline)
traj_m_spline = np.zeros((T_spline, 7))
from scipy.interpolate import UnivariateSpline

for m in range(modeNum):
    print('mode ', m)
    print('cost:', cost_m[m,0], 'collision:', cost_m[m,1])
    if cost_m[m,1] < 40.0:
        trajTask_opt_m = robKine.convertConfigTraj2TaskTraj(trajConfig_mulTrajOpt[m])

        fname_opt_m = foldername + taskname + 'OptPath_mode' + str(m) + '.csv'
        np.savetxt(fname_opt_m, trajConfig_mulTrajOpt[m], delimiter=',')

        fname_opt_task_m = foldername + taskname + 'OptPath_task_mode' + str(m) + '.csv'
        np.savetxt(fname_opt_task_m, trajTask_opt_m, delimiter=',')

        for i in range(7):
            s = UnivariateSpline(t, trajConfig_mulTrajOpt[m, :, i] )
            traj_m_spline[:, i] = s(ts)

        time.sleep(1)
        robCon.startSimulation()
        time.sleep(0.1)

        robCon.sendTargetTrajectoryConfig(traj_m_spline, signal_rate)
        time.sleep(0.1)

        time.sleep(1)
        robCon.stopSimulation()

time.sleep(2)
robCon.stopCommunication()
print ('Program ended')