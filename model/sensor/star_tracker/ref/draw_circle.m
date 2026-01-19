function img = draw_circle(img, x, y, radius, color)
% draw_circle - 이미지에 원 그리기
%
% Inputs:
%   img    - Background image
%   x, y   - Center coordinates
%   radius - Circle radius
%   color  - Circle color (intensity)
%
% Output:
%   img - Updated image

    [h, w] = size(img);

    % Create grid
    for u = max(1, x-radius):min(w, x+radius)
        for v = max(1, y-radius):min(h, y+radius)
            % Check if pixel is within circle
            dist = sqrt((u-x)^2 + (v-y)^2);
            if dist <= radius
                img(v, u) = color;
            end
        end
    end
end
