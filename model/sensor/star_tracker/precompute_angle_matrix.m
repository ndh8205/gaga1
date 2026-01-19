%% precompute_angle_matrix.m (NEW)
function angle_matrix = precompute_angle_matrix(b_vectors)
% 모든 별쌍 각거리 사전 계산 - O(n²) 1회
%
% Input:
%   b_vectors - 3xN
%
% Output:
%   angle_matrix - NxN (대칭)

N = size(b_vectors, 2);
angle_matrix = zeros(N, N);

for i = 1:N-1
    for j = i+1:N
        cos_angle = max(-1, min(1, b_vectors(:,i)' * b_vectors(:,j)));
        angle = acos(cos_angle);
        angle_matrix(i,j) = angle;
        angle_matrix(j,i) = angle;
    end
end
end