function [R, q] = quest_attitude_SVD(b_obs, r_cat, weights)
% QUEST with Newton-Raphson for λ_max only
% Quaternion extraction identical to SVD method
%
% Inputs:
%   b_obs   - 3xN (body frame)
%   r_cat   - 3xN (inertial frame)
%   weights - Nx1 (optional)
%
% Outputs:
%   R - 3x3 rotation matrix (I to B)
%   q - 4x1 quaternion [qw; qx; qy; qz]

N = size(b_obs, 2);

if nargin < 3
    weights = ones(N, 1);
end
weights = weights / sum(weights);

%% 1. Attitude profile matrix B
B = zeros(3, 3);
for i = 1:N
    B = B + weights(i) * (b_obs(:,i) * r_cat(:,i)');
end

%% 2. Parameters for characteristic polynomial
S = B + B';
s = trace(B);
Z = [B(2,3) - B(3,2); 
     B(3,1) - B(1,3); 
     B(1,2) - B(2,1)];

adjS = S(2,2)*S(3,3) - S(2,3)*S(3,2) + ...
       S(1,1)*S(3,3) - S(1,3)*S(3,1) + ...
       S(1,1)*S(2,2) - S(1,2)*S(2,1);

a = s^2 - adjS;
b = s^2 + Z'*Z;
c = det(S) + Z'*S*Z;
d = Z'*S*S*Z;

%% 3. Newton-Raphson for λ_max (partially factored)
lambda = 1.0;

for iter = 1:5
    psi = (lambda^2 - a)*(lambda^2 - b) - c*lambda + (c*s - d);
    psi_prime = 2*lambda*(lambda^2 - a) + 2*lambda*(lambda^2 - b) - c;
    lambda = lambda - psi / psi_prime;
end

%% 4. Quaternion via SVD (SAME AS ORIGINAL)
% Davenport K matrix
K = [S - s*eye(3), Z;
     Z',           s];

% Eigenvector of largest eigenvalue
[U, ~, V] = svd(B);
R = U * diag([1, 1, det(U*V')]) * V';

% DCM to quaternion
q = DCM2Quat(R);

end