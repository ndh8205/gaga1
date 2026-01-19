function [candidates, k_start, k_end] = kvector_range_search(target_angle, tolerance, catalog_data)
% kvector_range_search - k-vector로 각거리 범위 내 별쌍 검색
%
% Inputs:
%   target_angle  - 검색할 각거리 (radians)
%   tolerance     - 허용오차 (radians), 보통 k*sigma
%   catalog_data  - build_star_catalog_kvector 출력
%
% Outputs:
%   candidates - struct array with I, J indices
%   k_start, k_end - k-vector 인덱스

ya = target_angle - tolerance;
yb = target_angle + tolerance;

% k-vector 인덱스 계산
j_b = floor((ya - catalog_data.q) / catalog_data.m);
j_t = ceil((yb - catalog_data.q) / catalog_data.m);

% 범위 체크
j_b = max(1, min(j_b, catalog_data.N_pairs));
j_t = max(1, min(j_t, catalog_data.N_pairs));

k_start = catalog_data.k_vector(j_b) + 1;
k_end = catalog_data.k_vector(j_t);

% 후보 추출
if k_end >= k_start
    candidates = catalog_data.sorted_pairs(k_start:k_end);
else
    candidates = [];
end

% 경계 정리 (extraneous elements 제거)
if ~isempty(candidates)
    % 앞쪽 체크
    while ~isempty(candidates) && candidates(1).angle < ya
        candidates(1) = [];
        k_start = k_start + 1;
    end
    
    % 뒷쪽 체크
    while ~isempty(candidates) && candidates(end).angle > yb
        candidates(end) = [];
        k_end = k_end - 1;
    end
end
end