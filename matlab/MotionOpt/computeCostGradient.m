function [ grad_cost ] = computeCostGradient( x,  bodySize, obs, radius, eps)
%COMPUTECOSTGRADIENT Summary of this function goes here
%   Detailed explanation goes here

    n = size(x);
    
    c = computeCost(x, bodySize, obs, radius, eps);
    
    perturb_eps = 0.001;
    
    grad_cost = zeros( n );
    
    for i=1:n(2)
        x_dx = x;
        x_dx(i) = x_dx(i) + perturb_eps; 
        c_dc = computeCost(x_dx, bodySize, obs, radius, eps);
        grad_cost(i) = (c_dc - c)/perturb_eps;
    end

end

