%% identify_remaining_by_projection.m (재확인)
function [star_ids, indices] = identify_remaining_by_projection(R_I2B, b_remaining, remaining_idx, catalog, tolerance)
% Projection 기반 remaining star 식별
%
% Inputs:
%   R_I2B        - 3x3 자세 (I→B)
%   b_remaining  - 3xM 남은 별 벡터
%   remaining_idx - M 인덱스
%   catalog      - 카탈로그
%   tolerance    - 각도 threshold
%
% Outputs:
%   star_ids  - 식별된 카탈로그 ID
%   indices   - 관측 인덱스

M = size(b_remaining, 2);
star_ids = [];
indices = [];

% 관측→Inertial 변환
R_B2I = R_I2B';

for i = 1:M
    % 예측 방향 (Inertial)
    r_pred = R_B2I * b_remaining(:,i);
    
    % 전체 카탈로그와 비교 (vectorized)
    cos_angles = catalog.r_I' * r_pred;  % Nx1
    cos_angles = max(-1, min(1, cos_angles));
    
    % 최소 각도
    [max_cos, best_idx] = max(cos_angles);
    angle = acos(max_cos);
    
    % Threshold 체크
    if angle < tolerance
        star_ids = [star_ids; best_idx];
        indices = [indices; remaining_idx(i)];
    end
end

end