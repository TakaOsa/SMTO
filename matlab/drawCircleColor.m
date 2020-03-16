function h = drawCircleColor(x,y,r, color)
    hold on
%     th = 0:pi/50:2*pi;
%     xunit = r * cos(th) + x;
%     yunit = r * sin(th) + y;
%     h = plot(xunit, yunit, 'k');
    
    pos = [x - r, y- r, r*2, r*2 ]; 
    rectangle('Position',pos,'Curvature',[1 1], 'FaceColor', color, 'Linewidth', 0.1);
    %rectangle('Position',pos,'Curvature',[1 1], 'FaceColor', color, 'EdgeColor', 'none');
    %rectangle('Position',pos,'Curvature',[1 1], 'EdgeColor', color)
    
    hold off
end

