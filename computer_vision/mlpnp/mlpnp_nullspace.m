function [nullspaces, cov_reduced] = mlpnp_nullspace(bearings, cov_bearings)
% MLPnP null space 계산 (공분산 선택적 지원)
%
% Inputs:
%   bearings      - 3xN bearing vectors
%   cov_bearings  - 9xN bearing 공분산 (optional)
%
% Outputs:
%   nullspaces   - Nx1 cell array of 3x2 null space 행렬
%   cov_reduced  - 2x2xN reduced 공분산 (optional)

N = size(bearings, 2);
nullspaces = cell(N, 1);

% 공분산 처리 여부 확인
use_cov = nargout > 1 && nargin >= 2 && ~isempty(cov_bearings);

if use_cov
    cov_reduced = zeros(2, 2, N);
else
    cov_reduced = [];
end

for i = 1:N
    f = bearings(:, i);
    
    % C++는 정규화하지 않음!
    % f = f / norm(f);  % 제거!
    
    % SVD of f' (1x3 행렬)
    [~, ~, V] = svd(f', 0);  % full decomposition
    nullspaces{i} = V(:, 2:3);  % 마지막 2개 열 [r, s]
    
    % Reduced covariance 계산 (선택적)
    if use_cov
        % 3x3 공분산 복원
        Sigma_3x3 = reshape(cov_bearings(:,i), 3, 3);
        
        % Null space로 투영
        null_2d = nullspaces{i};  % 3x2 행렬
        
        % Reduced covariance: (N' * Σ * N)^-1
        % Urban 논문 Eq. (15) 참조
        cov_reduced(:,:,i) = inv(null_2d' * Sigma_3x3 * null_2d);
    end
end

end