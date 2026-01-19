function [bearings, points_3d, valid_idx] = mlpnp_validate_input_fixed(pixels, points3D, visibility, K, distortion, max_points)
% 개선된 MLPnP 입력 검증 - 실제 카메라 파라미터 사용
%
% Inputs:
%   pixels      - 2xN 픽셀 좌표
%   points3D    - 3xN 3D 포인트
%   visibility  - 1xN 가시성 플래그
%   K           - 3x3 카메라 내부 파라미터
%   distortion  - 왜곡 파라미터 구조체 (optional)
%   max_points  - 최대 포인트 수 (default: 30)
%
% Outputs:
%   bearings    - 3xM bearing vectors
%   points_3d   - 3xM 대응하는 3D 포인트
%   valid_idx   - 1xM 원본 인덱스

if nargin < 5
    distortion = [];
end
if nargin < 6
    max_points = 30;
end

% 가시적인 포인트만 선택
visible_idx = find(visibility > 0.5);
N_visible = length(visible_idx);

if N_visible < 6
    error('MLPnP: Minimum 6 visible points required. Current: %d', N_visible);
end

% 포인트 수 제한 (균등 샘플링)
if N_visible > max_points
    % 이미지 중심에서 거리 기반 샘플링
    cx = K(1,3);
    cy = K(2,3);
    
    vis_pixels = pixels(:, visible_idx);
    distances = sqrt((vis_pixels(1,:) - cx).^2 + (vis_pixels(2,:) - cy).^2);
    
    % 거리별로 균등 샘플링
    [~, sort_idx] = sort(distances);
    step = floor(N_visible / max_points);
    sample_idx = sort_idx(1:step:end);
    sample_idx = sample_idx(1:min(max_points, length(sample_idx)));
    
    valid_idx = visible_idx(sample_idx);
else
    valid_idx = visible_idx;
end

% 선택된 데이터
points_3d = points3D(:, valid_idx);
valid_pixels = pixels(:, valid_idx);

% Bearing vectors 계산
bearings = mlpnp_compute_bearings(valid_pixels, K, distortion);

% 정규화 확인
bearing_norms = vecnorm(bearings);
if any(abs(bearing_norms - 1) > 1e-6)
    bearings = bearings ./ bearing_norms;
end

end