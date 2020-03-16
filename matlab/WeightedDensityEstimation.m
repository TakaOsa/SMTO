% This demo shows the behavior of the density estimation with the
% cost-weighted importance.
% The result will be different for each run since the process with
% stochastic.

close all;
clear variables;

N = 1000;
m = 10;

Amax = 2;
RewardFunc = @ReturnTwoModes;

range = [ 0; 2; 0; Amax  ];
gridsize = 0.005;
SAset = rand(N, 2);
SAset(: , 1) = 2*SAset(: , 1);
SAset(: , 2) = Amax * SAset(: , 2);

Rset = zeros(N, 1);

for i=1:N
    Rset(i, 1) = RewardFunc( SAset( i, 1 ), SAset( i, 2 ) );
end

D = SAset;
D_laplace = LaplacianEigenMapping(D, 15, 5)';
x = D';
 
z_ini = mod( randperm(N), m ) + 1;


%===== with reward ==========
Rmax = max( Rset );
Rmin = min( Rset);
weights = exp( 5* (Rset - Rmax)/ (Rmax -Rmin) )' ;
weights = N * weights/sum(weights);

CostFunc = @CostTwoModes;
 z_rewardweight = IWVBEMGMM( D', m, z_ini, weights, N  );
 VisualizeReward( RewardFunc, range, gridsize ); axis( [0 2 0 Amax] ); 
 hold on;
 
 g = ( z_rewardweight == 1 );  X = x( : , g ); height = 2 * ones( sum(g), 1 ); scatter3( X(1, :), X( 2, : ), height,'MarkerEdgeColor','k','MarkerFaceColor',[1 0.3 0.3] );
 g = ( z_rewardweight == 2 );  X = x( : , g ); height = 2 * ones( sum(g), 1 ); scatter3( X(1, :), X( 2, : ), height,'MarkerEdgeColor','k','MarkerFaceColor',[0.3 0.3 1]  );
 g = ( z_rewardweight == 3 );  X = x( : , g ); height = 2 * ones( sum(g), 1 ); scatter3( X(1, :), X( 2, : ), height,'MarkerEdgeColor','k','MarkerFaceColor',[0.3 1 0.3]  );
 g = ( z_rewardweight == 4 );  X = x( : , g ); height = 2 * ones( sum(g), 1 ); scatter3( X(1, :), X( 2, : ), height,'MarkerEdgeColor','k','MarkerFaceColor',[1 1 0.3]  );
 g = ( z_rewardweight == 5 );  X = x( : , g ); height = 2 * ones( sum(g), 1 ); scatter3( X(1, :), X( 2, : ), height,'MarkerEdgeColor','k','MarkerFaceColor',[0.3 1 1]  );
 
 
 set(gca,'YTickLabel',[]);
 set(gca,'XTickLabel',[]);
 
 figure('Position', [100, 100, 800, 600]);
 for i=1:N
    if z_rewardweight(i) == 1
        color = [1 0.3 0.3];
    elseif z_rewardweight(i) == 2
        color = [0.3 0.3 1];
    elseif z_rewardweight(i) == 3
        color = [0.3 1 0.3];
    elseif z_rewardweight(i) == 4
        color = [1 1 0.3];
    else
        color = [0.3 1 1];
    end
        
    drawCircleColor( x(1, i), x( 2, i), 0.01*weights(1, i), color);
 end
 
 axis([0 2 0 2])
  set(gca,'YTickLabel',[]);
 set(gca,'XTickLabel',[]);
 
 figure('Position', [100, 100, 800, 600]);

 scatter(x(1,:), x(2,:), 'MarkerFaceColor',[0 .75 .75], 'MarkerEdgeColor','k');
 
 axis([0 2 0 2])
  set(gca,'YTickLabel',[]);
 set(gca,'XTickLabel',[]);
 
 VisualizeReward( RewardFunc, range, gridsize ); axis( [0 2 0 Amax] ); 
 axis([0 2 0 2])
 set(gca,'YTickLabel',[]);
 set(gca,'XTickLabel',[]);