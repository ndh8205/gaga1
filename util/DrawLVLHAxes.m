function DrawLVLHAxes(length, colors)
    line([0 length],[0 0],[0 0],'Color',colors.radial,'LineWidth',1.5); text(length,0,0,' X');
    line([0 0],[0 length],[0 0],'Color',colors.intrack,'LineWidth',1.5); text(0,length,0,' Y');
    line([0 0],[0 0],[0 length],'Color',colors.crosstrack,'LineWidth',1.5); text(0,0,length,' Z');
    % plot3(0,0,0, 'k+', 'MarkerSize', 15, 'LineWidth', 2);
end
