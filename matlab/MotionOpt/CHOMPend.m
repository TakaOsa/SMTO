function [trajConfig, collisionCost]  = CHOMPend( traj_q, T, funcJacob, dim, ... 
                                                    iteNum, obstacles, radii, eps, bodySizes, collision_threshold,...
                                                    update_rate, cost_weights, freeAxis_start, freeAxis_end  )

    linkNum = size(traj_q,2);
    compJacob = funcJacob;
    
    traj_task = zeros(T, linkNum, dim);
    J_traj = zeros(T, dim, linkNum);
    
    for t=1:T
       [x, ~] = compJacob( traj_q(t, :), linkNum );
             
       traj_task(t, :, :) = x;

    end
    
    traj_ini = traj_task;
    vel_task = zeros( T, linkNum, dim );
    accel_task = zeros( T, linkNum, dim );
    
    trajCSpaceIni = traj_q;
    velCspace = zeros( T, linkNum);
    accelCspace = zeros( T, linkNum);

    cost_t = zeros(T, linkNum);
    costGrad_t = zeros(T, linkNum, dim);
    
    collisionCost =100;
    collisionCost_pre = -1;
    cnt  = 0;
 
    
    K = zeros(T, T);
    for t=2:T-1
        K(t, t) = 1.0; 
        K(t+1, t) = - 1.0;
    end
    K(T, T)=1;
        

       
    A = K' * K;
    A(T,T)=2;
    
    K_st = zeros(T, T);
    for t=1:(T-1)
        K_st(t, t) = 1.0; 
        K_st(t+1, t) = - 1.0;
    end
    A_st = K_st' * K_st;
    M_st = pinv(A_st);
    
    M_end = pinv(A);
    
    cntMin = 15;
    
    
    %for iteration = 1:iteNum
    while collisionCost > collision_threshold || cnt < cntMin
        cnt =  cnt + 1;
        if cnt > iteNum
            disp('maximum iteration number.')
            break;
        end
            
        %Compute the Jacobian for each link at each time step
        for t=1:T
            [x, J] = compJacob( traj_q(t, :), linkNum );
            traj_task(t, :, :) = x;
            if dim ==2
                J_traj(t, :, :) = J(1:dim, :);
            end
        end
        
        for i=2:T
            vel_task(i,:, :) = traj_task(i, :, :) - traj_task(i-1, :, :);
            velCspace(i,:) = traj_q(i, :) - traj_q(i-1, :); 
        end

        for i=2:T
            accel_task(i,:, :) = vel_task(i,:, :) - vel_task(i-1, :, :);
            accelCspace(i,:, :) = velCspace(i,:) - velCspace(i-1, :);
        end


        %Compute the collision cot and its gradient
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
        
        if abs(collisionCost - collisionCost_pre) < 1e-3 && cnt >= cntMin
            disp('CHOMP converged.')
            break;
        end
        
        F_grad_obs =  zeros(T, linkNum);
        I = eye(dim, dim);

        boundary = zeros(T, linkNum);
        boundary(2, :) = - traj_q(1, :);
      
        bodyIdx = linkNum;
        
        vel = zeros(T, dim);
        vel(:, :) = vel_task(:, bodyIdx, :);
        
        accel = zeros(T, dim);
        accel(:, :) = accel_task(:, bodyIdx, :);
        
        goalCost = zeros(T, linkNum);        
        F_grad_smooth =  K' * K * traj_q + boundary;
        
        for t=2:(T-1)
            
            [xt, Jt]= compJacob( traj_q(t, :), bodyIdx);
            J = zeros(dim, linkNum);
            J(:, :) = Jt(1:dim, :);

            costgrad = zeros(dim, 1);
            costgrad(:) = costGrad_t(t, bodyIdx, :);            
            projection =  I - vel(t,:)' * vel(t,:) / norm(vel(t,:))^2;

            Ka = projection * accel(t, :)' / norm(vel(t,:))^2;
            F_grad_obs(t, :) = J' * norm(vel(t,:))* ( projection* costgrad - cost_t(t, bodyIdx) * Ka);
            
        end

        update =  pinv(A) * (cost_weights(1) * F_grad_obs - cost_weights(2) * F_grad_smooth);
        
        traj_q = traj_q - update_rate * update;
        collisionCost_pre = collisionCost;
        
        %============updating the starting point=======================
        traj_base = traj_q;
        noise_ang = pi*1./180;
        noise_endRot = noise_ang * freeAxis_start;
        noise_endRot = [ zeros(1, 3), noise_endRot ];
        [~, J] = compJacob(traj_q(1, :), linkNum);
        noise_endRot_q = pinv(J) * noise_endRot';
        noise_end_traj = zeros(linkNum, T);
        noise_end_traj(:, 1) = noise_endRot_q;
        
        noise_end_prop = M_st * noise_end_traj';
        noise_end_prop = noise_end_prop * (noise_end_prop(1, 3) / noise_endRot_q(3, 1));
        
        cost_temp = [];
        traj_temp = [];
        
        for c= 1:3
            if c == 1
                traj_test = traj_base;
            elseif c== 2
                traj_test = traj_base + noise_end_prop;
                
            else 
                traj_test = traj_base - noise_end_prop;
            end
            
            %Compute the Jacobian for each link at each time step
            for t=1:T
                [x, ~] = compJacob( traj_test(t, :), linkNum );
                traj_task(t, :, :) = x;
                %J_traj(t, :, :) = J;
            end

            %Compute the collision cost
            for b = 1:linkNum
                for i=1:T
                    bodyPoint = zeros(1, dim);
                    bodyPoint(1, :) = traj_task(i, b, :);
                    cost_t(i, b) = computeCost( bodyPoint, bodySizes(b), obstacles, radii, eps );
                    %costGrad_t(i, b, :) = computeCostGradient( bodyPoint, bodySizes(b), obstacles, radii, eps );
                end
            end
            
            collisionCost =  sum( sum(cost_t) );
            smoothnessCost = trace( (traj_q' * K') * (K * traj_q));
            total_cost = cost_weights(1) *collisionCost+  cost_weights(2) * smoothnessCost;
            
            if isempty(cost_temp)
                cost_temp =  total_cost;
                traj_temp = traj_test;
            elseif cost_temp > total_cost
                cost_temp = total_cost;
                traj_temp = traj_test;
            end
        end
        
        traj_q = traj_temp;
        %============ end of updating the starting point=======================
        
        %============updating the end point=======================
        traj_base = traj_q;
        noise_ang = pi*1./180;
        noise_endRot = noise_ang * freeAxis_end;
        noise_endRot = [ zeros(1, 3), noise_endRot ];
        [~, J] = compJacob(traj_q(T, :), linkNum);
        noise_endRot_q = pinv(J) * noise_endRot';
        noise_end_traj = zeros(linkNum, T);
        noise_end_traj(:, T) = noise_endRot_q;
        
        noise_end_prop = M_end * noise_end_traj';
        noise_end_prop = noise_end_prop * (noise_end_prop(T, 3) / noise_endRot_q(3, 1));
        
        cost_temp = [];
        traj_temp = [];
        
        for c= 1:3
            if c == 1
                traj_test = traj_base;
                traj_test_prop = traj_base;
            elseif c== 2
                traj_test = traj_base + noise_end_prop;

            else 
                traj_test = traj_base - noise_end_prop;

            end
            
            %Compute the Jacobian for each link at each time step
            for t=1:T
                [x, ~] = compJacob( traj_test(t, :), linkNum );
                traj_task(t, :, :) = x;
                %J_traj(t, :, :) = J;
            end

            %Compute the collision cost
            for b = 1:linkNum
                for i=1:T
                    bodyPoint = zeros(1, dim);
                    bodyPoint(1, :) = traj_task(i, b, :);
                    cost_t(i, b) = computeCost( bodyPoint, bodySizes(b), obstacles, radii, eps );
                    %costGrad_t(i, b, :) = computeCostGradient( bodyPoint, bodySizes(b), obstacles, radii, eps );
                end
            end
            
            collisionCost =  sum( sum(cost_t) );
            smoothnessCost = trace( (traj_q' * K') * (K * traj_q));
            cost_total = collisionCost+ smoothnessCost;
            
            if isempty(cost_temp) || cost_temp > cost_total
                cost_temp = cost_total;
                traj_temp = traj_test;

            end
        end
        
        traj_q = traj_temp;
        %============ end of updating the end point=======================
        
        %============ shit trajectory to the initial goal=======================
        [x, ~] = compJacob( traj_q(T, :), linkNum );
        error = zeros(dim, 1);

        error(:, 1) =  reshape(traj_ini(T,linkNum, :), [dim, 1]) -  reshape(x(linkNum, :), [dim, 1]);
        shift = zeros(6, 1);
        shift(1:dim, 1) = error(:);
        [~, J] = compJacob(traj_q(T, :), linkNum);
        shift_end_q = pinv(J) * shift;
        shift_end_traj = zeros(linkNum, T);
        shift_end_traj(:, T) = shift_end_q;
        shift_end_prop = M_end * shift_end_traj';
        
        shift_end_prop = shift_end_prop * (shift_end_prop(T, 3) / shift_end_q(3, 1));
         traj_q = traj_q + 0.5 * shift_end_prop;
        
        %disp(collisionCost);
    end

    trajConfig = traj_q;
    smoothnessCost = trace( (traj_q' * K') * (K * traj_q));
    fprintf('collisionCost:%f, smoothnessCost;%f, update num: %d \n',collisionCost, smoothnessCost, cnt);

end

