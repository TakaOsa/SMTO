clear variables;
close all;

addpath('MotionOpt');
addpath('FourLink');

T = 40; %number of step
D = 2;   %dimension of the state

linkNum = 4;
L = [1.0, 1.0, 1.0, 1.0]; % length of the link
basePos = [ 0, 0 ]; % position of the base

q_ini = [ 1.3*pi, -0.5*pi, -0.55*pi, 0.05*pi];
x_ini = ForwardKinematicsFourLink(q_ini,basePos, L );
x_end = [.5, 3.];

traj_task = ones(T, D);
for i = 1:40
   traj_task(i,:) = x_ini + (x_end - x_ini) * (1 - cos( (i-1) / (T-1) *pi ) )/2;
end

% Shift the trajectory so that the mean trajectory starts from q_ini
shift = x_ini - traj_task(1, :);
Ti = size(traj_task, 1);
traj_task = traj_task + repmat(shift, Ti, 1);
    
traj_c = InverseKinematicsFourLink(traj_task, q_ini);

%======== Obstacle settings =======
obsNum = 1;
obstacles = [ 0., 2.15];
radii = [ 0.2];

eps = 0.15;
bodySizes = [0.01, 0.15, 0.2,  0.15];

traj_q = traj_c;

stomp_cost_weights = [1.0, 0.1];
iteNum = 4;
seedNum = 400;
freeAxis = [0,0,1];
angle_pos =  pi/5;
angle_neg = pi/5;
angle_pos_st =  pi/8;
angle_neg_st = pi/10;
jointLimit_low = [-pi, -pi, -pi, -pi];
jointLimit_high = [ pi, pi, pi, pi];

%----- main function of SMTO -----
tic

[traj_SMTO, costSet]  = SMTOend( traj_q, T, @ComputeJacobianFourLinkRot,...
                            D, iteNum, obstacles, radii, eps, bodySizes, 0.15, stomp_cost_weights, angle_pos, angle_neg, ...
                        freeAxis, jointLimit_low, jointLimit_high, seedNum);
                                      
toc

modeNum = size(traj_SMTO, 3);

chomp_cost_weights = [1.0, 0.001];
for m=1:modeNum

    
    f4 = figure;
    for i =1:obsNum
        drawCircle(obstacles(i, 1), obstacles(i, 2),radii(i) );
    end
    for t =1:T
         grayScale = (1 - t/T);
        [ xc, J  ] = DrawFourLink( basePos, L, traj_SMTO(t, :, m), f4, grayScale ); 
        task_ini(t, :) =  xc(3, :);
    end

    drawCircleColor(x_end(1), x_end(2), 0.05, [0.5, 0, 0.5]);
    axis([-1.5 3 -1. 4])

    set(gca,'xtick',[]);
    set(gca,'ytick',[]);
    axis equal;
end

f1 = figure;
for i =1:obsNum
    drawCircle(obstacles(i, 1), obstacles(i, 2),radii(i) );
end
for t =[1,T]
    grayScale = (1 - t/T);
    DrawFourLink( basePos, L, traj_q(t, :), f1, grayScale ); 
end
drawCircleColor(x_end(1), x_end(2), 0.05, [0.5, 0, 0.5]);
axis([-1.5 3 -1.0 4])
set(gca,'xtick',[]);
set(gca,'ytick',[]);
axis equal;

rmpath('MotionOpt');
rmpath('FourLink');