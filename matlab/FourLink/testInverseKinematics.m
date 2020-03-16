clear all;

T = 100;
D = 2;

L = [0.1, 0.1, 0.1, 0.1]; % length of the link
basePos = [ 0, 0 ]; % position of the base

q_ini = [0.*pi, -0.25*pi, -0.25*pi, -0.25*pi];

trajTaskSpace = zeros(T, D);

for t =1:T
    trajTaskSpace(t, 1) = 0.1 + 0.0015*t; 
    trajTaskSpace(t, 2) = -0.2414; 
end

trajConfig = InverseKinematicsFourLink(trajTaskSpace, q_ini);

f1 = figure;

for t =1:T
   [ xc, J  ] = DrawFourLink( basePos, L, trajConfig(t, :), f1, 0.5 );    
end
   