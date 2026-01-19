function b_ST = pixel_to_unit_vector(pixel_coords, f, myu, l, w)
% pixel_to_unit_vector - 픽셀 좌표를 Star Tracker frame 단위벡터로 변환
%
% Inputs:
%   pixel_coords - Nx2 [x_pixel, y_pixel]
%   f            - 초점거리 (m)
%   myu          - 픽셀 크기 (m)
%   l, w         - 이미지 크기 (pixels)
%
% Output:
%   b_ST         - 3xN 단위벡터 (ST frame)

N = size(pixel_coords, 1);
b_ST = zeros(3, N);

for i = 1:N
    % 픽셀 → 센서 좌표
    x_sensor = pixel_coords(i,1) * myu;
    y_sensor = pixel_coords(i,2) * myu;
    
    % 방향 벡터
    b = [x_sensor; y_sensor; f];
    
    % 정규화
    b_ST(:,i) = b / norm(b);
end
end