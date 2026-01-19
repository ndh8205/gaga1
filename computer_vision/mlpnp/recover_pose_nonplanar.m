%% recover_pose_nonplanar 수정 (C++ 정확히 따라가기)
function [R, t] = recover_pose_nonplanar(x, points3D, bearings, nullspaces)
% C++와 100% 일치

N = size(points3D, 2);

% 행렬 구성
tmp = [x(1), x(4), x(7);
       x(2), x(5), x(8);
       x(3), x(6), x(9)];

% 스케일
col_norms = [norm(tmp(:,1)), norm(tmp(:,2)), norm(tmp(:,3))];
scale = 1.0 / (abs(prod(col_norms))^(1/3));

% SVD로 회전행렬
[U, ~, V] = svd(tmp);
Rout = U * V';
if det(Rout) < 0
    Rout = -Rout;
end

% 변환 벡터
tout = Rout * (scale * [x(10); x(11); x(12)]);

% C++ 방식: inverse 변환 사용!
n_test = min(6, N);
error = zeros(2, 1);

for s = 1:2
    % 4x4 변환 행렬 구성
    T = eye(4);
    T(1:3, 1:3) = Rout;
    if s == 1
        T(1:3, 4) = tout;
    else  
        T(1:3, 4) = -tout;
    end
    
    % C++: Ts[s] = Ts[s].inverse()
    T = inv(T);
    
    % 오차 계산
    for p = 1:n_test
        % 정규화된 bearing
        f_norm = bearings(:,p) / norm(bearings(:,p));
        
        % C++: v = Ts[s].block<3,3> * points3v[p] + Ts[s].block<3,1>
        v = T(1:3, 1:3) * points3D(:,p) + T(1:3, 4);
        v = v / norm(v);
        
        error(s) = error(s) + (1.0 - dot(v, f_norm));
    end
end

% 최소 오차 선택
if error(1) < error(2)
    % s=0 선택: T는 이미 inverse된 상태
    T_final = eye(4);
    T_final(1:3, 1:3) = Rout;
    T_final(1:3, 4) = tout;
    T_inv = inv(T_final);
    
    R = T_inv(1:3, 1:3);
    t = T_inv(1:3, 4);
else
    % s=1 선택
    T_final = eye(4);
    T_final(1:3, 1:3) = Rout;
    T_final(1:3, 4) = -tout;
    T_inv = inv(T_final);
    
    R = T_inv(1:3, 1:3);
    t = T_inv(1:3, 4);
end
end