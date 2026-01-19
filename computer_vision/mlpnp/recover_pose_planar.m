
function [R, t] = recover_pose_planar(x, points3D, bearings, eigenRot, nullspaces)
% C++와 정확히 일치

N = size(points3D, 2);

% Transform points
points_transformed = eigenRot * points3D;

% 부분 회전 행렬
tmp = zeros(3, 3);
tmp(1,2) = x(1); tmp(1,3) = x(2);
tmp(2,2) = x(3); tmp(2,3) = x(4);
tmp(3,2) = x(5); tmp(3,3) = x(6);

% 첫 번째 열 = col2 × col3
tmp(:,1) = cross(tmp(:,2), tmp(:,3));
tmp = tmp';  % transpose

% 스케일
scale = 1.0 / sqrt(abs(norm(tmp(:,2)) * norm(tmp(:,3))));

% SVD
[U, ~, V] = svd(tmp);
Rout1 = U * V';

if det(Rout1) < 0
    Rout1 = -Rout1;
end

% 원래 프레임으로
Rout1 = eigenRot' * Rout1;

% 변환 벡터
t = scale * [x(7); x(8); x(9)];

% C++ 변환
Rout1 = Rout1';
Rout1 = -Rout1;
if det(Rout1) < 0
    Rout1(:,3) = -Rout1(:,3);
end

% 4가지 해
R1 = Rout1;
R2 = Rout1;
R2(:,1:2) = -R2(:,1:2);

Ts{1} = {R1, t};
Ts{2} = {R1, -t};
Ts{3} = {R2, t};
Ts{4} = {R2, -t};

% 최적 선택
normVal = zeros(4, 1);
n_test = min(6, N);

for i = 1:4
    for p = 1:n_test
        reproPt = Ts{i}{1} * points3D(:,p) + Ts{i}{2};
        reproPt = reproPt / norm(reproPt);
        normVal(i) = normVal(i) + (1.0 - dot(reproPt, bearings(:,p)));
    end
end

[~, idx] = min(normVal);
R = Ts{idx}{1};
t = Ts{idx}{2};
end