%% ===== 5. rodrigues2rot.m =====
function R = rodrigues2rot(omega)
% C++와 일치

R = eye(3);
theta = norm(omega);

if theta > eps
    k = omega / theta;
    K = [0, -k(3), k(2);
         k(3), 0, -k(1);
         -k(2), k(1), 0];
    
    R = eye(3) + sin(theta) * K + (1 - cos(theta)) * (K * K);
end
end