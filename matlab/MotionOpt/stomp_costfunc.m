function [cost_total,collisionCost, smoothnessCost] = stomp_costfunc(traj_q, dim, funcJacob,obstacles, radii, eps, bodySizes, cost_weights)
%STOMP_COSTFUNC compute the cost function for STOMP

    compJacob = funcJacob;
    [T, linkNum] = size(traj_q);
    cost_t = zeros(T, linkNum);
    traj_task = zeros(T, linkNum, dim);

    %Compute the trajectory in the task space
    for t=1:T
        [x, ~] = compJacob( traj_q(t, :), linkNum );
        traj_task(t, :, :) = x;
    end
        
    %Compute the collision cost
    for i=1:T
        for b = 1:linkNum
            bodyPoint = zeros(1, dim);
            bodyPoint(1, :) = traj_task(i, b, :);
            cost_t(i, b) = computeCost( bodyPoint, bodySizes(b), obstacles, radii, eps );
        end
    end
        
    collisionCost =  sum( sum(cost_t) );
        
    %Smoothness cost
    K = zeros(T, T);
    for t=1:(T-1)
        K(t, t) = 1.0; 
        K(t+1, t) = - 1.0;
    end
       
    A = K' * K;
    
    velCspace = K * traj_q;
    accelCspace = A * traj_q;
    smoothnessCost = sum(sum( velCspace.^2 + accelCspace.^2  ));
        
    cost_total = cost_weights(1)*collisionCost + cost_weights(2)*smoothnessCost;
end

