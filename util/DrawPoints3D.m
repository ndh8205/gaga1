function DrawPoints3D(points, color)
    % 포인트를 작은 점으로 표시
    plot3(points(1,:), points(2,:), points(3,:), '.', ...
          'Color', color, 'MarkerSize', 2);
end