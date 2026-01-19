function [R_L2G, t, Cov_pose, statistics] = mlpnp_with_covariance(points3D, pixels, K, pixel_cov, max_iter)
% MLPnP with optional covariance weighting
%
% Outputs 수정:
%   R_L2G      - 3x3 rotation matrix (world to camera)
%   t          - 3x1 translation vector (world to camera)
%   Cov_pose   - 6x6 pose covariance [omega(3); t(3)]
%   statistics - Optimization statistics struct

% 입력 검증
if nargin < 3
    error('At least points3D, pixels, and K are required');
end

if nargin < 4
    pixel_cov = [];
end

if nargin < 5
    max_iter = 5;
end

% 픽셀 공분산 처리 및 정규화
use_cov = ~isempty(pixel_cov);
N = size(pixels, 2);

if use_cov
    % 다양한 입력 형식 처리
    if isscalar(pixel_cov)
        pixel_cov_matrix = pixel_cov * eye(2);
        pixel_cov = repmat(pixel_cov_matrix, [1, 1, N]);
        
    elseif isvector(pixel_cov) && length(pixel_cov) == 2
        pixel_cov_matrix = diag(pixel_cov);
        pixel_cov = repmat(pixel_cov_matrix, [1, 1, N]);
        
    elseif size(pixel_cov, 1) == 2 && size(pixel_cov, 2) == 2
        if size(pixel_cov, 3) == 1
            pixel_cov = repmat(pixel_cov, [1, 1, N]);
        elseif size(pixel_cov, 3) ~= N
            error('pixel_cov dimension mismatch: expected 2x2xN');
        end
    else
        error('Invalid pixel_cov format');
    end
end

% ========== Step 1: Bearing vectors 계산 ==========
if use_cov
    [bearings, cov_bearings] = mlpnp_compute_bearings(pixels, K, [], pixel_cov);
else
    bearings = mlpnp_compute_bearings(pixels, K);
    cov_bearings = [];
end

% ========== Step 2: Null space 계산 ==========
if use_cov
    [nullspaces, cov_reduced] = mlpnp_nullspace(bearings, cov_bearings);
else
    nullspaces = mlpnp_nullspace(bearings);
    cov_reduced = [];
end

% ========== Step 3: Design matrix 구성 ==========
if use_cov
    [A, is_planar, eigenRot, points_transformed, Kll] = mlpnp_design_matrix(points3D, nullspaces, cov_reduced);
else
    [A, is_planar, eigenRot, points_transformed] = mlpnp_design_matrix(points3D, nullspaces);
    Kll = [];
end

% ========== Step 4: 초기 해 계산 ==========
if use_cov
    [R_init, t_init] = mlpnp_solve_improved(A, is_planar, points3D, bearings, nullspaces, eigenRot, Kll);
else
    [R_init, t_init] = mlpnp_solve_improved(A, is_planar, points3D, bearings, nullspaces, eigenRot);
end

% ========== Step 5: Gauss-Newton 정밀화 + 공분산 ==========
if use_cov
    [R_L2G, t, Cov_pose, sigma_0] = mlpnp_gauss_newton(R_init, t_init, points3D, bearings, nullspaces, max_iter, Kll);
else
    [R_L2G, t, Cov_pose, sigma_0] = mlpnp_gauss_newton(R_init, t_init, points3D, bearings, nullspaces, max_iter);
end

% ========== Statistics 계산 ==========
if nargout > 3
    statistics = compute_statistics(R_L2G, t, Cov_pose, sigma_0, points3D, bearings, use_cov, Kll);
end

end

% ========================================================================
% Helper function: 통계 정보 계산
% ========================================================================
function stats = compute_statistics(R, t, Cov_pose, sigma_0, points3D, bearings, use_cov, Kll)
    N = size(points3D, 2);
    
    % Reprojection errors
    errors = zeros(N, 1);
    for i = 1:N
        p_cam = R * points3D(:,i) + t;
        p_cam_norm = p_cam / norm(p_cam);
        
        % Angular error (radians)
        errors(i) = acos(min(1, max(-1, dot(p_cam_norm, bearings(:,i)))));
    end
    
    stats.mean_error = mean(errors);
    stats.std_error = std(errors);
    stats.max_error = max(errors);
    stats.rms_error = sqrt(mean(errors.^2));
    
    % 포즈 불확실성 (논문 Eq. 27)
    if ~isempty(Cov_pose)
        % Standard deviations
        sigma_pose = sigma_0 * sqrt(diag(Cov_pose));  % 6x1
        
        stats.sigma_rotation = sigma_pose(1:3);  % [rad]
        stats.sigma_translation = sigma_pose(4:6);  % [km or m]
        stats.sigma_0 = sigma_0;  % Variance factor
        
        % 공분산 행렬 전체 저장
        stats.Cov_pose = Cov_pose;  % 6x6
    else
        stats.sigma_rotation = [];
        stats.sigma_translation = [];
        stats.sigma_0 = [];
        stats.Cov_pose = [];
    end
    
    stats.num_points = N;
    stats.covariance_used = use_cov;
end