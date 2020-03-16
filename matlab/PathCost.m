function [ R ] = PathCost( s, a )
%PathCost outputs the reward based on TWO optimal linear lines

%a = 0.2 * s + 1.2; 

x = s;
if s > 1
    x = 2 - s;
end
y = a;

% d1 = abs(  0.4 * x - y  + 1.0  ) / sqrt(  0.4^2 + 1.0^2  );
% 
% %a = - 0.3 * s + 0.9; 
% d2 = abs(  - 0.4 * x - y + 1.0 ) / sqrt(  0.4^2 + 1.0^2  );
% 
% d = d1;
% if d1 > d2 
%     d = d2;
% end
% 
% R = exp( - d/5  );

R = 1.0;

% if y < 1.1 && s > 0.7 && s < 1.3 && y > 0.9
%     R = 0.3;
% end

if y < 0.4 *x + 0.8 && y > -0.4 * x + 1.2 
    R = - 0.1;
end

if y > 0.4 *x + 1.5 || y < - 0.4 *x + 0.5
    R = -0.1;
end 


