function h = drawCircle(x,y,r)
    hold on
%     th = 0:pi/50:2*pi;
%     xunit = r * cos(th) + x;
%     yunit = r * sin(th) + y;
%     h = plot(xunit, yunit, 'k');
    
    pos = [x - r, y- r, r*2, r*2 ]; 
    rectangle('Position',pos,'Curvature',[1 1], 'FaceColor',[0 .5 .5])
    
    
    hold off
end

