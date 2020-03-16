function [ ori] = ComputeOriFourLink( q )
%ComputeJacobianThreeLink Summary of this function goes here
%   Detailed explanation goes here

    ori = zeros(3, 1);

    ori(1,1) = cos( q(1) + q(2) + q(3) + q(4));
    ori(2,1) = sin( q(1) + q(2) + q(3) + q(4));
    ori(3,1) = 0;
    
end

