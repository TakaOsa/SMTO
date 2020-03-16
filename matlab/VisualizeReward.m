function [ Rmap ] = VisualizeReward( rewardFunc, range, gridsize )
%VISUALIZEREWARD 
% range should be [  s_min, s_max, a_min, a_max  ]

S =  range(1,1):gridsize:range(2,1);
A = range(3,1):gridsize:range(4,1);

M = size(S,2);
N = size(A,2);
Rmap = zeros( N , M  );

for i = 1:N
    for j=1:M
        Rmap( i, j) = rewardFunc( S(j), A(i) );
    end
end

f = figure('Position', [100, 100, 800, 600]);

Rmax = max( max(Rmap) );
%Rmin = min( min(Rmap) )
%contour(S, A, Rmap);

surf( S, A, Rmap - Rmax );
shading interp;


end

