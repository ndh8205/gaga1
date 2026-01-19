function angles = compute_interstar_angles(b_vectors)
% compute_interstar_angles - 관측된 별들의 각거리 계산
%
% Input:
%   b_vectors - 3xN 단위벡터
%
% Output:
%   angles - struct array with fields i, j, angle

N = size(b_vectors, 2);
count = 0;
max_count = N * (N-1) / 2;
angles = struct('i', cell(max_count,1), 'j', cell(max_count,1), 'angle', cell(max_count,1));

for i = 1:N-1
    for j = i+1:N
        cos_angle = b_vectors(:,i)' * b_vectors(:,j);
        cos_angle = max(-1, min(1, cos_angle));
        
        count = count + 1;
        angles(count).i = i;
        angles(count).j = j;
        angles(count).angle = acos(cos_angle);
    end
end
end