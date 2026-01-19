
function DrawSatelliteBody(R_body_in_lvlh, pos, scale, colors)
    axes_world = R_body_in_lvlh * (scale * eye(3));
    
    line([pos(1) pos(1)+axes_world(1,1)], [pos(2) pos(2)+axes_world(2,1)], [pos(3) pos(3)+axes_world(3,1)], 'Color', colors.sat_body_x, 'LineWidth', 3);
    line([pos(1) pos(1)+axes_world(1,2)], [pos(2) pos(2)+axes_world(2,2)], [pos(3) pos(3)+axes_world(3,2)], 'Color', colors.sat_body_y, 'LineWidth', 3);
    line([pos(1) pos(1)+axes_world(1,3)], [pos(2) pos(2)+axes_world(2,3)], [pos(3) pos(3)+axes_world(3,3)], 'Color', colors.sat_body_z, 'LineWidth', 3);
    scatter3(pos(1), pos(2), pos(3), 100, colors.deputy, 'filled', 'MarkerEdgeColor', 'k');
end
