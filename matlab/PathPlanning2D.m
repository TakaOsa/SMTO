% ---- Problem setting ----
% you can play around by channgin the problem setting.
% solutions are different for each run since the process is stochastic

% position of the start and goal points
%p_start = [0,1.5];
%p_goal = [2,1];

p_start = [0,1.3];
p_goal = [2, 0.7];

% shape of the cost function 
% please use one of "PathCost", "PathCost1", "PathCost2" and "PathCost3"
RewardFunc = @PathCost1;

T = 50;
N = 4000;
m = 10;
path_ini = zeros(T, 2);

for i =1:T
   path_ini(i, :) = p_start + (i-1)/(T-1) * (p_goal - p_start);
end

K = zeros(T-1, T-1);
for t=1:(T-2)
    K(t, t) = 1.0; 
    K(t+1, t) = - 1.0;
end

M = K' * K;
R = pinv(M);
A = 10 * R / (max(max(R)) * T);
zero_mean = zeros(1,T-1);

PathSet = zeros(T, 2, N);
Rset = zeros(N,1);
SolSet = [];

for k =1:1

    for n=1:N
        noise = mvnrnd(zero_mean, A, 2);
        zeros_point = zeros(2,1);
        noise = [zeros_point, noise];
        if k ==1
            PathSet(:,:,n) =  noise.' + path_ini;
        else
            L = size(SolSet, 3);
            t = mod(n, L);
            PathSet(:,:,n) =  noise.' + SolSet(:,:,t+1);
        end
        Rset(n) = EvalPath(PathSet(:,:,n), RewardFunc); 
    end

    PathData = reshape(PathSet, [T*2, N]);

    % Dimensionality reduction with Laplacian Eigenmap
    D = PathData';
    D_laplace = LaplacianEigenMapping(D, 15, 5)';

    % Scale the importance weight
    Rmax = max( Rset );
    Rmin = min( Rset);
    weights = exp( 10* (Rset - Rmax)/ (Rmax -Rmin) )' ;
    weights = N * weights/sum(weights);
    z_ini = mod( randperm(N), m ) + 1;
    z_rewardweight = IWVBEMGMM( D_laplace, m, z_ini, weights, N  );
    modes = max(z_rewardweight);
    SolSet= zeros(T,2,modes);

    for j=1:modes
       ind = (z_rewardweight==j);
       x_j = D(ind, :);
       weights_j = weights(ind);
       ind_w = (weights_j < 0.5);
       weights_j(ind_w) = 0;
       n_j = sum(weights_j);
       weight_tile = repmat(weights_j.', 1, T*2);
       mean_j = sum(weight_tile .* x_j) / n_j;
       path_j = reshape(mean_j, [T, 2]);
       SolSet(:,:,j) = path_j;
    end

end

Amax = 2;
range = [ -0.2; 2.2; 0; Amax  ];
gridsize = 0.001;

% Visualization of the obtained solutions
VisualizeReward( RewardFunc, range, gridsize ); axis( [-0.2 2.2 0 Amax] ); 
hold on;
 
height = ones(T,1);
 for j=1:modes
    ret_j = EvalPath(SolSet(:,:,j), RewardFunc)
    if ret_j > 49
        plot3(SolSet(:,1,j), SolSet(:,2,j), height, '-o');
    end
 end

% Visualization of the explored paths
VisualizeReward( RewardFunc, range, gridsize ); axis( [-0.2 2.2 0 Amax] ); 
hold on;
 
height = ones(T,1);
 for j = 1:30
    plot3(PathSet(:,1,j), PathSet(:,2,j), height);
 end
 
set(gca,'xtick',[])
set(gca,'ytick',[])
 

