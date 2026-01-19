%% kvector_range_search_fast.m (NEW)
function [I_arr, J_arr, angle_arr] = kvector_range_search_fast(target_angle, tolerance, catalog)
% 배열 기반 고속 검색
%
% Outputs:
%   I_arr, J_arr, angle_arr - 배열 (구조체 X)

ya = target_angle - tolerance;
yb = target_angle + tolerance;

j_b = floor((ya - catalog.q) / catalog.m);
j_t = ceil((yb - catalog.q) / catalog.m);

j_b = max(1, min(j_b, catalog.N_pairs));
j_t = max(1, min(j_t, catalog.N_pairs));

k_start = double(catalog.k_vector(j_b)) + 1;
k_end = double(catalog.k_vector(j_t));

if k_end < k_start
    I_arr = [];
    J_arr = [];
    angle_arr = [];
    return;
end

% 배열 추출
I_arr = catalog.pairs_I(k_start:k_end);
J_arr = catalog.pairs_J(k_start:k_end);
angle_arr = catalog.pairs_angle(k_start:k_end);

% 경계 필터링 (vectorized)
valid = (angle_arr >= ya) & (angle_arr <= yb);
I_arr = I_arr(valid);
J_arr = J_arr(valid);
angle_arr = angle_arr(valid);

end