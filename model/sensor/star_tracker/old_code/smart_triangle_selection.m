%% smart_triangle_selection.m (NEW)
function triangle_list = smart_triangle_selection(n, magnitudes, angle_matrix, max_triangles)
% Magnitude 밝은 별 + 큰 각거리 우선
%
% Inputs:
%   n            - 별 개수
%   magnitudes   - Nx1 magnitude (작을수록 밝음)
%   angle_matrix - NxN 각거리
%   max_triangles - 최대 시도 개수
%
% Output:
%   triangle_list - Mx3 [i,j,k]

if nargin < 4
    max_triangles = 10;  % 기본값
end

% 1. Magnitude 정렬
[~, bright_idx] = sort(magnitudes, 'ascend');

% 2. 밝은 별 중심으로 triangle 생성
triangle_list = [];
triangle_scores = [];

% 가장 밝은 n개 중에서 조합
n_bright = min(n, 6);  % 상위 6개만 고려

for m = 1:n_bright-2
    for n2 = m+1:n_bright-1
        for p = n2+1:n_bright
            i = bright_idx(m);
            j = bright_idx(n2);
            k = bright_idx(p);
            
            % Triangle quality score
            % 큰 각거리 + 밝은 별 = 높은 점수
            mean_angle = (angle_matrix(i,j) + angle_matrix(i,k) + angle_matrix(j,k)) / 3;
            mean_mag = (magnitudes(i) + magnitudes(j) + magnitudes(k)) / 3;
            
            score = mean_angle * 10 - mean_mag;  % 각거리 가중치 높임
            
            triangle_list = [triangle_list; i, j, k];
            triangle_scores = [triangle_scores; score];
        end
    end
end

% 3. Score 기준 정렬
[~, sort_idx] = sort(triangle_scores, 'descend');
triangle_list = triangle_list(sort_idx, :);

% 4. 최대 개수 제한
triangle_list = triangle_list(1:min(max_triangles, size(triangle_list,1)), :);

end