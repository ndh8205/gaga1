
function DrawParticles2D(pixels, visible, color)
    vis_idx = find(visible);
    if ~isempty(vis_idx)
        scatter(pixels(1,vis_idx), pixels(2,vis_idx), 100, color, 'filled');
        for i = vis_idx
            text(pixels(1,i), pixels(2,i), sprintf('%d', i), ...
                'Color', 'w', 'FontSize', 9, 'FontWeight', 'bold', 'HorizontalAlignment', 'center');
        end
    end
end