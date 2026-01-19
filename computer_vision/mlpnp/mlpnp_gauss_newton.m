function [R_refined, t_refined, Cov_pose, sigma_0] = mlpnp_gauss_newton(R_init, t_init, points3D, bearings, nullspaces, max_iter, Kll)
% MLPnP Gauss-Newton 최적화 (해석적 자코비안 버전)
%
% Outputs 추가:
%   Cov_pose - 6x6 포즈 공분산 [omega(3); t(3)]
%   sigma_0  - Variance factor (aposteriori sigma)

if nargin < 6
    max_iter = 5;
end

% 가중치 처리
use_weights = nargin >= 7 && ~isempty(Kll);

N = size(points3D, 2);

% Rodriguez 파라미터로 변환
omega = rot2rodrigues(R_init);
x = [omega; t_init];

% 최적화 파라미터
eps_p = 1e-5;
stop = false;
iter = 0;

% 최종 J, r 저장용
J_final = [];
r_final = [];

while iter < max_iter && ~stop
    % Residuals와 Jacobian 계산 (해석적 버전)
    [r, J] = compute_residuals_jacobian_analytical(x, points3D, nullspaces);
    
    % 최종 iteration 값 저장
    J_final = J;
    r_final = r;
    
    % Normal equations
    if use_weights
        % 가중 최소제곱법
        JtKllJ = J' * Kll * J;
        JtKllr = J' * Kll * r;
        dx = JtKllJ \ JtKllr;
    else
        % 일반 최소제곱법
        JtJ = J' * J;
        Jtr = J' * r;
        dx = JtJ \ Jtr;
    end
    
    % 발산 방지
    if max(abs(dx)) > 5.0 || any(isnan(dx)) || any(isinf(dx))
        break;
    end
    
    % 업데이트
    dl = J * dx;
    if max(abs(dl)) < eps_p
        stop = true;
        x = x - dx;
    else
        x = x - dx;
    end
    
    iter = iter + 1;
end

% 결과 변환
omega_refined = x(1:3);
t_refined = x(4:6);
R_refined = rodrigues2rot(omega_refined);

% ========== 포즈 공분산 계산 (논문 Eq. 23, 27) ==========
if nargout > 2
    if use_weights && ~isempty(J_final) && ~isempty(r_final)
        % Hessian (Normal matrix)
        H = J_final' * Kll * J_final;
        
        % 공분산 행렬
        Cov_pose = inv(H);
        
        % Variance factor (aposteriori)
        redundancy = 2*N - 6;  % 2N observations - 6 unknowns
        if redundancy > 0
            sigma_0 = sqrt((r_final' * Kll * r_final) / redundancy);
        else
            sigma_0 = 1.0;  % 최소 자유도
        end
    else
        % 가중치 없는 경우
        H = J_final' * J_final;
        Cov_pose = inv(H);
        redundancy = 2*N - 6;
        if redundancy > 0
            sigma_0 = sqrt((r_final' * r_final) / redundancy);
        else
            sigma_0 = 1.0;
        end
    end
else
    Cov_pose = [];
    sigma_0 = [];
end

end

function [r, J] = compute_residuals_jacobian_analytical(x, points3D, nullspaces)
% (변경 없음)
omega = x(1:3);
t = x(4:6);
R = rodrigues2rot(omega);

N = size(points3D, 2);
r = zeros(2*N, 1);
J = zeros(2*N, 6);

for i = 1:N
    p_cam = R * points3D(:,i) + t;
    p_cam_norm = p_cam / norm(p_cam);
    
    ns = nullspaces{i};
    r(2*i-1) = ns(:,1)' * p_cam_norm;
    r(2*i)   = ns(:,2)' * p_cam_norm;
    
    jac = jacobians_Rodrigues(points3D(1,i), points3D(2,i), points3D(3,i), ...
                             ns(1,1), ns(2,1), ns(3,1), ...
                             ns(1,2), ns(2,2), ns(3,2), ...
                             t(1), t(2), t(3), ...
                             omega(1), omega(2), omega(3));
    
    J(2*i-1:2*i, :) = jac;
end
end