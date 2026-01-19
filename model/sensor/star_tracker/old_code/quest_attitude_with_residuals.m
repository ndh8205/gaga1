%% quest_attitude_with_residuals.m (NEW)
function [R, q, residuals] = quest_attitude_with_residuals(b_obs, r_cat, weights)
% QUEST + residual 계산
%
% Outputs:
%   residuals - Nx1 각도 오차 (rad)

N = size(b_obs, 2);

if nargin < 3
    weights = ones(N, 1);
end
weights = weights / sum(weights);

%% QUEST
B = zeros(3, 3);
for i = 1:N
    B = B + weights(i) * (b_obs(:,i) * r_cat(:,i)');
end

[U, ~, V] = svd(B);
R = U * diag([1, 1, det(U*V')]) * V';
q = DCM2Quat(R);

%% Residuals
residuals = zeros(N, 1);
for i = 1:N
    r_pred = R * r_cat(:,i);
    cos_angle = max(-1, min(1, b_obs(:,i)' * r_pred));
    residuals(i) = acos(cos_angle);
end

end