function [R] = PathCost3(s,a)

%PathCost outputs the reward based on TWO optimal linear lines

R = 1.0;

if s > 0.5 && s < 1.1 && a > 0.7 && a < 1.2
    R = 0;
end

if s > 0.7 && s < 1.1 && a > 0 && a < 0.3
    R = 0;
end

if s > 1.4 && s < 1.6 && a > .6 && a < 0.8
    R = 0;
end
    
if s > 0.5 && s < .7 && a > 1.5 && a < 2.5
    R = 0;
end

if s > 1.5 && s < 1.7 && a > 1.5 && a < 2.5
    R = 0;
end

end