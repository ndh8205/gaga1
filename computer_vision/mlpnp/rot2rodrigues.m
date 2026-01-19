
%% ===== 6. rot2rodrigues.m =====
function omega = rot2rodrigues(R)
% C++와 일치

omega = zeros(3, 1);

trace_R = trace(R) - 1.0;
theta = acos(max(-1, min(1, trace_R / 2.0)));

if theta > eps
    % C++ 방식
    omega(1) = (R(3,2) - R(2,3));
    omega(2) = (R(1,3) - R(3,1));
    omega(3) = (R(2,1) - R(1,2));
    sc = theta / (2.0 * sin(theta));
    omega = omega * sc;
end
end