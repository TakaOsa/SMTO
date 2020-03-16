function [ c ] = computeCost( x, bodySize,  obstacles, radii, eps )
%COMPUTECOST Summary of this function goes here
%   Detailed explanation goes here
    
    N =  size(obstacles, 1);
    
    
    diff =  obstacles' - repmat(x', 1, N);
    distance =  sqrt( dot(diff, diff) ) - radii' - bodySize;
    
    dim = size(x, 2);
%     if dim == 3
%        distance = [distance, x(1, 3)]; 
%     end
    
    [disMin, idxMin] = min( distance );

    obs = obstacles( idxMin, : );
    d = disMin;
    radius = radii(idxMin);
    
    %eps = 1.0;
    %d = sqrt( sum((obs - x).^2) ) - radius

    
    
    if d < 0
        c = -d + eps*0.5;
    
    else if d < eps
            
            c = 0.5  * (d - eps)^2 / eps;
                        
        else
            
            c = 0;
            
        end
    end
    
end

