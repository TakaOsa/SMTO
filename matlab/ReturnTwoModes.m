function [ R ] = CostTwoModes( s, a )
%REWARDDOUBLE outputs the reward based on TWO optimal linear lines

a1 = s* sin(-pi*20/180) + a*cos(-pi*20/180);
s1 = s* cos(-pi*20/180) - a*sin(-pi*20/180);

%(0.2, 1.2)
d1 = sqrt((1.0 - s1)^2 + 4*(1.2 - a1)^2);

a2 = s* sin(pi*15/180) + a*cos(pi*15/180);
s2 = s* cos(pi*15/180) - a*sin(pi*15/180);

%(0.2, 1.2)
d2 = sqrt(2*(1.2 - s2)^2 + (0.7 - a2)^2);

d = d1;
if d1 > d2 
    d = d2;
end

R = exp( - d /10 );

end

