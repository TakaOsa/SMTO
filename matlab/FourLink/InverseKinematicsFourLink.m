function [ trajConfig ] = InverseKinematicsFourLink( trajTaskSpace, q_ini )
%InverseKinematicsThreeLink Summary of this function goes here
%   Detailed explanation goes here

    T = size(trajTaskSpace, 1);
    linkNum = size(q_ini, 2);
    
    trajConfig = zeros(T, linkNum);
    
    trajConfig(1, :) =  q_ini;
    
    for t= 2:T
        diff = zeros(2, 1);
        diff(:) = trajTaskSpace(t, :) - trajTaskSpace(t-1, :);
        
        q_next = zeros(1, linkNum);
        q_next(1, :) = trajConfig(t-1, :);
        
        for ite = 1:2
            [xt, Jt] = ComputeJacobianFourLink( q_next, linkNum );

            diff_q = pinv(Jt) * diff;

            q_next = q_next + diff_q';
            diff(:) = trajTaskSpace(t, :) - xt(linkNum, :);
        end
        
        trajConfig(t, :) = q_next;
    end


end

