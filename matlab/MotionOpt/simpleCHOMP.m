function [trajConfig, collisionCost, trajTask, trajQset]  = simpleCHOMP( traj_q, T, funcJacob, dim, ... 
                                                    iteNum, obstacles, radii, eps, bodySizes, collision_threshold, update_rate, cost_weights  )
%function traj_opt  = simpleKineCHOMP( traj_q, T, L, basePos, funcJacob, dim,  iteNum, obstacles, radii, eps  )
%simpleKineCHOMP Summary of this function goes here
%   Detailed explanation goes here

    linkNum = size(traj_q,2);
    compJacob = funcJacob;
    
    trajQset =  zeros(T, linkNum, iteNum);
    trajQset(:, :, 1) = traj_q;
    
    traj_task = zeros(T, linkNum, dim);
    J_traj = zeros(T, 6, linkNum);
    
    for t=1:T
       [x, J] = compJacob( traj_q(t, :), linkNum );
       %[x, J] = ComputeJacobianThreeLink(basePos, L, traj_q(t, :) );
       
       traj_task(t, :, :) = x;
       J_traj(t, :, :) = J;
    end
    
    
    traj_ini = traj_task;
    vel_task = zeros( T, linkNum, dim );
    accel_task = zeros( T, linkNum, dim );
    
    trajCSpaceIni = traj_q;
    velCspace = zeros( T, linkNum);
    accelCspace = zeros( T, linkNum);
    
    %jerk = zeros( T, dim );

    cost_t = zeros(T, linkNum);
    costGrad_t = zeros(T, linkNum, dim);
    
    collisionCost =100;
    collisionCost_pre = -1;
    cnt  = 0;
    
    %         K = zeros(T-1, T);
%         for t=1:(T-1)
%            K(t, t) = - 1.0; 
%            K(t, t+1) = 1.0;
%         end
        
%     K = zeros(T, T);
%     for t=1:(T-1)
%         K(t, t) = 1.0; 
%         K(t+1, t) = - 1.0;
%     end
    
    K = zeros(T, T);
    for t=2:T-1
        K(t, t) = 1.0; 
        K(t+1, t) = - 1.0;
    end
    K(T, T)=1;
        
    %vel = zeros( T, dim );
    %vel = K * trajectory;
       
    A = K' * K;
%      A(T-1, T) = 1;
%      A(1,1)=0;
%      A(1,2)=0;
    A(T,T)=2;
    
    
    for iteration = 1:iteNum
    %while collisionCost > collision_threshold
        cnt =  cnt + 1;
        
        for t=1:T
            [x, J] = compJacob( traj_q(t, :), linkNum );
            traj_task(t, :, :) = x;
            J_traj(t, :, :) = J;
        end
        
        for i=2:T
            vel_task(i,:, :) = traj_task(i, :, :) - traj_task(i-1, :, :);
            velCspace(i,:) = traj_q(i, :) - traj_q(i-1, :); 
        end

        for i=2:T
            accel_task(i,:, :) = vel_task(i,:, :) - vel_task(i-1, :, :);
            accelCspace(i,:, :) = velCspace(i,:) - velCspace(i-1, :);
        end



        for b = 1:linkNum
            for i=1:T
                bodyPoint = zeros(1, dim);
                bodyPoint(1, :) = traj_task(i, b, :);
                
                cost_t(i, b) = computeCost( bodyPoint, bodySizes(b), obstacles, radii, eps );
                costGrad_t(i, b, :) = computeCostGradient( bodyPoint, bodySizes(b), obstacles, radii, eps );
            end
        end
        
        bodyCollision = sum( cost_t, 1 );
        [maxCollision, bodyIdx] = max(bodyCollision);
        
        if maxCollision < 1e-2
            bodyIdx =  linkNum;
        end

        collisionCost =  sum( sum(cost_t) );
        
        if abs(collisionCost - collisionCost_pre) < 1e-4
            disp('CHOMP converged.')
            break;
        end
        
        F_grad_obs =  zeros(T, linkNum);
        I = eye(dim, dim);

        %F_grad_smooth =  zeros(T, linkNum);
        boundary = zeros(T, linkNum);
        boundary(2, :) = - traj_q(1, :);
%         boundary(1, :) = - traj_q(1, :);
%         boundary(T, :) = traj_q(T, :);
        
        %F_grad_demo =  zeros(T, dim);
        
        bodyIdx = linkNum;
        
        vel = zeros(T, dim);
        vel(:, :) = vel_task(:, bodyIdx, :);
        
        accel = zeros(T, dim);
        accel(:, :) = accel_task(:, bodyIdx, :);
        
        goalCost = zeros(T, linkNum);        
        F_grad_smooth =  K' * K * traj_q + boundary;
        %F_grad_smooth =  zeros(T, linkNum);  
        
        %F_grad_smooth = accelCspace;
        %F_grad_demo =  trajCSpaceIni - traj_q ;
        
        for t=2:(T-1)
            
            [xt, Jt]= compJacob( traj_q(t, :), bodyIdx);

            costgrad = zeros(dim, 1);
            costgrad(:) = costGrad_t(t, bodyIdx, :);            
            projection =  I - vel(t,:)' * vel(t,:) / norm(vel(t,:))^2;

            Ka = projection * accel(t, :)' / norm(vel(t,:))^2;
            if dim ==2
               Jt = Jt(1:2, :); 
            end
                
            F_grad_obs(t, :) = Jt' * norm(vel(t,:))* ( projection* costgrad - cost_t(t, bodyIdx) * Ka);
            
        end

        update =  pinv(A) * (cost_weights(1) * F_grad_obs - cost_weights(2) * F_grad_smooth);
        
        traj_q = traj_q - update_rate * update;
        collisionCost_pre = collisionCost;
        %disp(collisionCost);
    end
    dev_traj = traj_q - trajCSpaceIni;
    dev = sum( sum(dev_traj.^2, 2), 1);
    
    trajConfig = traj_q;
    trajTask = traj_task;
    smoothnessCost = trace( (traj_q' * K') * (K * traj_q));
    fprintf('collisionCost:%f, smoothnessCost;%f, update num: %d \n',collisionCost, smoothnessCost, cnt);
    %traj_opt = [ traj_ini(1, :);  trajectory; traj_ini(T, :) ];
end

