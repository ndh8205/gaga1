function [bearings, cov_bearings] = mlpnp_compute_bearings(pixels, K, distortion, pixel_cov)
% MLPnP bearing vector 계산 (공분산 선택적 지원)
% 
% Inputs:
%   pixels     - 2xN 픽셀 좌표
%   K          - 3x3 카메라 내부 행렬
%   distortion - 왜곡 계수 (optional, 현재 미지원)
%   pixel_cov  - 픽셀 측정 공분산 (optional)
%                2x2 행렬 (모든 점 동일) 또는
%                2x2xN (각 점별 다른 공분산)
%
% Outputs:
%   bearings     - 3xN bearing vectors
%   cov_bearings - 9xN bearing 공분산 (reshape된 3x3 행렬들)

N = size(pixels, 2);
bearings = zeros(3, N);

cx = K(1,3); cy = K(2,3);
fx = K(1,1); fy = K(2,2);

% Bearing vector 계산 (기존 로직 유지)
if nargin < 3 || isempty(distortion)
    for i = 1:N
        x = (pixels(1,i) - cx) / fx;
        y = (pixels(2,i) - cy) / fy;
        z = 1.0;
        
        % C++ 방식: z로 나누기만 (정규화 안함)
        bearings(:,i) = [x/z; y/z; 1];  % [x, y, 1]
    end
else
    error('Distortion not implemented');
end

% 공분산 계산 (선택적)
if nargout > 1 && nargin >= 4 && ~isempty(pixel_cov)
    cov_bearings = zeros(9, N);
    
    % Jacobian: pixel → bearing 변환
    % bearing = K^-1 * [u; v; 1] 에서 K^-1이 Jacobian
    J = [1/fx,    0,   -cx/fx;
           0,  1/fy,   -cy/fy;
           0,    0,       1];
    
    % 2x2 또는 2x2xN 공분산 처리
    if size(pixel_cov, 3) == 1
        % 모든 점이 동일한 공분산
        Sigma_pixel_3x3 = [pixel_cov, zeros(2,1); 
                           zeros(1,2), 0];
        Sigma_bearing = J * Sigma_pixel_3x3 * J';
        
        for i = 1:N
            cov_bearings(:,i) = reshape(Sigma_bearing, 9, 1);
        end
    else
        % 각 점별 다른 공분산
        for i = 1:N
            Sigma_pixel_3x3 = [pixel_cov(:,:,i), zeros(2,1);
                               zeros(1,2), 0];
            Sigma_bearing = J * Sigma_pixel_3x3 * J';
            cov_bearings(:,i) = reshape(Sigma_bearing, 9, 1);
        end
    end
else
    % 공분산 요청 없음
    cov_bearings = [];
end

end