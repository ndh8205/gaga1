
function DrawParticles3D(positions, color)
    num = size(positions, 2);
    scatter3(positions(1,:), positions(2,:), positions(3,:), 80, color, 'filled');
    for i = 1:num
        text(positions(1,i), positions(2,i), positions(3,i) + 0.01, sprintf('%d', i), ...
            'Color', 'k', 'FontSize', 8, 'FontWeight', 'bold', 'HorizontalAlignment', 'center');
    end
end
