function [ x, J ] = DrawFourLinkColor( basePos, L, q , figHandle, grayScale, color)
%DrawThreeLink Summary of this function goes here
%   Detailed explanation goes here

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
    
    figure(figHandle);
    x_pos = [ basePos; x  ];
    line(x_pos(:,1), x_pos(:,2), 'Color',grayScale*color);
    hold on;
    plot(basePos(1, 1), basePos(1, 2), 'o', 'Color',grayScale*color);
    plot(x(1, 1), x(1, 2), 'o', 'Color',grayScale*color);
    plot(x(2, 1), x(2, 2), 'o', 'Color',grayScale*color);
    plot(x(3, 1), x(3, 2), 'o', 'Color',grayScale*color);
    plot(x(4, 1), x(4, 2), 'o', 'Color',grayScale*color);
   
    
    drawnow;
end

