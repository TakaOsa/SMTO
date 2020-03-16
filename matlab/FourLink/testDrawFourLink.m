
f1 = figure;

T =100;
L = [0.1, 0.1, 0.1, 0.1];
basePos = [ 0, 0 ];

q = zeros(T, 4);

for t=1:T
    q (t, 1) = pi*0.5 - pi * 0.005*t;
    q (t, 2) = - pi * 0.0025*t;
    q (t, 3) = - pi * 0.0025*t;
    q (t, 4) = - pi * 0.0025*t;
end

x = zeros(T, 2);

for t =1:T
   [ xc, J  ] = DrawFourLink( basePos, L, q(t, :), f1, 0.5 ); 
   
   x(t, :) = xc(4, :);
   
   if t > 1
       x_d = x(t, :) - x(t-1, :);
       qd = q(t, :) - q(t-1, :);
       x_dot = J* qd';
   end
   
end