function [traj_hto, cost_opt_set] = SMTO(traj_q, T, funcJacob, dim, ... 
                        iteNum, obstacles, radii, eps, bodySizes, collision_threshold, cost_weights)
%function for stochastic multimodal trajectory optimization (SMTO)

    compJacob = funcJacob;
    linkNum = size(traj_q,2);
    traj_task = zeros(T, linkNum, dim);

    for t=1:T
       [x, ~] = compJacob( traj_q(t, :), linkNum );
       traj_task(t, :, :) = x;
    end
        
    trajCSpaceIni = traj_q;
    velCspace = zeros( T, linkNum);
    accelCspace = zeros( T, linkNum);
    
    cost_t = zeros(T, linkNum);
    
    K = zeros(T, T);

    for t=2:T-1
        K(t, t) = 1.0; 
        K(t+1, t) = - 1.0;
    end
    K(T, T)=1;
       
    A = K' * K;
    A(T,T)=2;



    B = A;
    B( T, : ) = [ ];
    B( : , T ) = [ ];
    B(:, 1) = [ ];
    R = inv(B' * B);

    maxR = max(max(R));
    M =1.5* R / (maxR *T);
    zero_mean = zeros( T-2, 1 );

    N = 600;
    modeNum = 1;
    bestBatchNum = 100;
    collisionCost =100;
    collisionCost_pre = 100;
    cnt  = 0;
    traj_hto = [];
    
    noise_set =  zeros(T-2, linkNum, N);
    cost_set = zeros(N, 1);
    p_set = zeros(N, 1);
    traj_sample_set = zeros(T, linkNum, N);
    best_traj_sample_set = zeros(T, linkNum, bestBatchNum);
    best_traj_costs = 100*ones(bestBatchNum, 1);
    p_ini = 1;
    
    for iteration = 1:iteNum
        %evaluate the current trajectory
        [cost_total,collisionCost, smoothnessCost] = stomp_costfunc(traj_q, dim, funcJacob,obstacles, radii, eps, bodySizes, cost_weights);
        
        for k = 1:N
            
            %Add the noise and evaluate
            noise = mvnrnd( zero_mean', M, linkNum);
            p = mvnpdf(noise,zero_mean',M); 
            if iteration == 1 && k==1
                p_ini = p(1);
            end
            p_set(k, 1) = prod(p/p_ini);
            
            zero_ini = zeros(1, linkNum);
            traj_noise = [zero_ini; noise'; zero_ini];
            
            if iteration == 1
                traj_q_noise= traj_q + traj_noise;
            else
                m = mod(k, modeNum)+1;
                traj_q_noise = traj_hto(:,:,m) + traj_noise;
            end
                                
            [cost_total,~, ~] = stomp_costfunc(traj_q_noise, dim, funcJacob,obstacles, radii, eps, bodySizes, cost_weights);
            noise_set(:,:, k) = noise';
            cost_set(k, 1) = cost_total;
            traj_sample_set(:,:,k) = traj_q_noise;
        end
        
        D =[];
        for k=1:N
           D = [D; reshape( traj_sample_set(:,:,k), [ 1, T*linkNum ]) ]; 
        end
        
        exp_cost_set = exp( - 50 *(cost_set - min(cost_set))/ (max(cost_set)- min(cost_set)));
        exp_cost_set = exp_cost_set / mean(exp_cost_set);
        
        m = 10;
        z_ini = mod( randperm(N), m ) + 1;
        D_laplace = LaplacianEigenMapping(D, 30, 9)';
        
        z = IWVBEMGMM( D_laplace, m, z_ini, exp_cost_set', 1000  );
        
        modeNum = max(z);
        traj_hto = zeros(T, linkNum, modeNum);
        for m=1:modeNum
           traj_mode_m = zeros(T, linkNum);
           ind = (z==m);
           num = sum(ind);
           traj_sample_set_m = traj_sample_set(:,:,ind);
           traj_sample_set_m_2d = reshape( traj_sample_set_m, [T*linkNum, num] );
           exp_cost_set_m  = exp_cost_set(ind, :);
           exp_cost_set_m_2d = repmat(exp_cost_set_m', [T*linkNum, 1]); 
           mean_traj_m = sum( exp_cost_set_m_2d .*  traj_sample_set_m_2d, 2) / sum( exp_cost_set_m );
           traj_hto(:, :, m) =  reshape( mean_traj_m, [T, linkNum] );
        end
                 
    end
    
    cost_opt_set = zeros(modeNum, 1);
    for m = 1:modeNum
       [cost_total_m,~, ~] = stomp_costfunc(traj_hto(:,:,m), dim, funcJacob,obstacles, radii, eps, bodySizes, cost_weights); 
       cost_opt_set(m, 1) = cost_total_m;
       
        smoothnessCost = trace( (traj_hto(:,:,m)' * K') * (K * traj_hto(:,:,m)));
        fprintf('cost_total:%f, smoothnessCost;%f\n',cost_total_m, smoothnessCost);
    end

end

