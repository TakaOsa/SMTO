function [R] = PathCost1(s,a)

%PathCost outputs the reward based on TWO optimal linear lines

R = 1.0;

if s > 0.7 && s < 1.1 && a > 1.0 && a < 1.4
    R = 0;
end

if s > .1 && s < .3 && a > 1.4 && a < 2
    R = 0;
end

if s > 1.4 && s < 1.6 && a > .6 && a < 1.2
    R = 0;
end
    
if s > 0. && s < .7 && a > 0 && a < 0.5
    R = 0;
end

if s > 1.5 && s < 1.7 && a > 1.5 && a < 2.5
    R = 0;
end

end