function DrawPoints2D(pixels, visible, color)
    if any(visible)
        visible_pixels = pixels(:, visible);
        % 2D에서도 작은 점으로 표시
        plot(visible_pixels(1,:), visible_pixels(2,:), '.', ...
             'Color', color, 'MarkerSize', 2);
    end
end