function [valid_bearings, valid_points, valid_idx] = mlpnp_validate_input(pixels, points3D, visibility, max_points)
% MLPnP: 입력 데이터 검증 및 샘플링
%
% Inputs:
%   pixels      - 2xN 픽셀 좌표
%   points3D    - 3xN 3D 포인트
%   visibility  - 1xN 가시성 플래그
%   max_points  - 최대 포인트 수 (default: 30)
%
% Outputs:
%   valid_bearings - 3xM 유효한 bearing vectors
%   valid_points   - 3xM 대응하는 3D 포인트
%   valid_idx      - 1xM 원본 인덱스

if nargin < 4
    max_points = 30;  % 기본값
end

% 가시적인 포인트만 선택
visible_idx = find(visibility > 0.5);
N_visible = length(visible_idx);

if N_visible < 6
    error('MLPnP: 최소 6개의 가시적인 포인트가 필요합니다. 현재: %d', N_visible);
end

% 포인트 수 제한 (균등 샘플링)
if N_visible > max_points
    % 균등하게 분포된 인덱스 선택
    step = N_visible / max_points;
    sample_idx = round(1:step:N_visible);
    sample_idx = unique(sample_idx);
    
    % 정확히 max_points개 선택
    if length(sample_idx) > max_points
        sample_idx = sample_idx(1:max_points);
    end
    
    valid_idx = visible_idx(sample_idx);
else
    valid_idx = visible_idx;
end

% 선택된 데이터
valid_points = points3D(:, valid_idx);
valid_pixels = pixels(:, valid_idx);

% 카메라 내부 파라미터 추정 (시뮬레이션에서 제공된 값 사용)
% 실제로는 camera_params.intrinsic.K_ideal 사용
image_width = 1024;
image_height = 1024;
fov = 5.5 * pi/180;
f = image_width / (2 * tan(fov/2));
K = [f, 0, image_width/2;
     0, f, image_height/2;
     0, 0, 1];

% Bearing vectors 계산 (왜곡 없다고 가정)
valid_bearings = mlpnp_compute_bearings(valid_pixels, K, []);

fprintf('MLPnP: %d/%d 포인트 선택됨\n', length(valid_idx), N_visible);

end