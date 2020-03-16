function [ x, J] = ComputeJacobianFourLink( q, linkIdx  )
%ComputeJacobianThreeLink Summary of this function goes here
%   Detailed explanation goes here

    L = [1, 1, 1, 1.]; % length of the link
    %L = [0.1, 0.1, 0.1, 0.1]; % length of the link
    basePos = [ 0, 0 ]; % position of the base
    
    J = zeros(2, 4);

    J(1,1) = - L(1) * sin( q(1) ) - L(2) * sin( q(1) + q(2) ) - L(3) * sin(q(1) + q(2) + q(3))  - L(4) * sin(q(1) + q(2) + q(3) + q(4));
    J(2,1) = L(1) * cos( q(1) ) + L(2) * cos( q(1) + q(2) ) + L(3) * cos(q(1) + q(2) + q(3)) +  L(4) * cos(q(1) + q(2) + q(3) + q(4));
    
    J(1,2) = - L(2) * sin( q(1) + q(2) ) - L(3) * sin(q(1) + q(2) + q(3))  - L(4) * sin(q(1) + q(2) + q(3) + q(4));
    J(2,2) = L(2) * cos( q(1) + q(2) ) + L(3) * cos(q(1) + q(2) + q(3)) +  L(4) * cos(q(1) + q(2) + q(3) + q(4));
    
    J(1,3) = - L(3) * sin(q(1) + q(2) + q(3)) - L(4) * sin(q(1) + q(2) + q(3) + q(4));
    J(2,3) =  L(3) * cos(q(1) + q(2) + q(3)) +  L(4) * cos(q(1) + q(2) + q(3) + q(4)); 
    
    J(1,4) = - L(4) * sin(q(1) + q(2) + q(3) + q(4));
    J(2,4) =  L(4) * cos(q(1) + q(2) + q(3) + q(4)); 
    
    x = zeros(4, 2);
    
    x(1,1) = basePos(1,1) + L(1) * cos( q(1) );
    x(1,2) = basePos(1,2) + L(1) * sin( q(1) );
    
    x(2,1) = x(1,1) + L(2) * cos( q(1) + q(2) );
    x(2,2) = x(1,2) + L(2) * sin( q(1) + q(2) );
    
    x(3,1) = x(2,1) + L(3) * cos( q(1) + q(2) + q(3) );
    x(3,2) = x(2,2) + L(3) * sin( q(1) + q(2) + q(3) );
    
    x(4,1) = x(3,1) + L(4) * cos( q(1) + q(2) + q(3) + q(4) );
    x(4,2) = x(3,2) + L(4) * sin( q(1) + q(2) + q(3) + q(4) );
    
end

