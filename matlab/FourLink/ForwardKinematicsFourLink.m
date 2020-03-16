function [ traj_c ] = ForwardKinematicsFourLink( traj_q, basePos, L )
%FORWARDKINEMATICSTHREELINK Summary of this function goes here
%   Detailed explanation goes here

    T = size(traj_q, 1);

    traj_c = zeros(T, 2);
    for i = 1:T
        q = zeros(1, 4);
        q(:) = traj_q(i, :);
        
        x = zeros(4, 2);

        x(1,1) = basePos(1,1) + L(1) * cos( q(1) );
        x(1,2) = basePos(1,2) + L(1) * sin( q(1) );

        x(2,1) = x(1,1) + L(2) * cos( q(1) + q(2) );
        x(2,2) = x(1,2) + L(2) * sin( q(1) + q(2) );

        x(3,1) = x(2,1) + L(3) * cos( q(1) + q(2) + q(3) );
        x(3,2) = x(2,2) + L(3) * sin( q(1) + q(2) + q(3) );
        
        x(4,1) = x(3,1) + L(4) * cos( q(1) + q(2) + q(3) + q(4) );
        x(4,2) = x(3,2) + L(4) * sin( q(1) + q(2) + q(3) + q(4));
        
        traj_c(i, :) = x(4, :);
    end
end

