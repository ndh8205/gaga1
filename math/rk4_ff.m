function [p_new, q_new, v_new, omega_new, theta_new, theta_dot_new] = ...
    rk4_ff(p_base, q_base, v_base, omega_base, theta, theta_dot, tau_func, t, dt, params)
% rk4_ff_old_v2: 예전 코드용 RK4 적분기 (기준점 수정)
%
% 핵심: H, c는 장착점(p_mount) 기준으로 정의됨
%       위성 CoM 속도 ↔ 장착점 속도 변환 필요
%
% 입력:
%   p_base, v_base: 위성 CoM 위치/속도 (관성좌표계)
%   omega_base: 위성 각속도 (body frame)

% 상태 패킹
state = pack_state(p_base, q_base, v_base, omega_base, theta, theta_dot);

% k1
tau1 = tau_func(t);
k1 = state_derivative(p_base, q_base, v_base, omega_base, theta, theta_dot, tau1, params);

% k2
s2 = state + 0.5*dt*k1;
[p2, q2, v2, omega2, theta2, theta_dot2] = unpack_state(s2);
q2 = q2 / norm(q2);
tau2 = tau_func(t + 0.5*dt);
k2 = state_derivative(p2, q2, v2, omega2, theta2, theta_dot2, tau2, params);

% k3
s3 = state + 0.5*dt*k2;
[p3, q3, v3, omega3, theta3, theta_dot3] = unpack_state(s3);
q3 = q3 / norm(q3);
tau3 = tau_func(t + 0.5*dt);
k3 = state_derivative(p3, q3, v3, omega3, theta3, theta_dot3, tau3, params);

% k4
s4 = state + dt*k3;
[p4, q4, v4, omega4, theta4, theta_dot4] = unpack_state(s4);
q4 = q4 / norm(q4);
tau4 = tau_func(t + dt);
k4 = state_derivative(p4, q4, v4, omega4, theta4, theta_dot4, tau4, params);

% 상태 업데이트
state_new = state + (dt/6)*(k1 + 2*k2 + 2*k3 + k4);

% 언패킹
[p_new, q_new, v_new, omega_new, theta_new, theta_dot_new] = unpack_state(state_new);
q_new = q_new / norm(q_new);

end

%% 상태 미분 계산 (위성 CoM 기준)
function ds = state_derivative(p_base, q_base, v_base, omega_base, theta, theta_dot, tau, params)
    
    % 회전행렬
    R_base = GetDCM_QUAT(q_base);
    
    % omega_base: body frame → inertial frame
    omega_inertial = R_base * omega_base(:);
    
    % FK (위성 CoM 기준)
    FK = RA_FK(params, p_base, q_base, theta);
    
    % Mass Matrix (13x13, 위성 CoM 기준)
    H = RA_MM(FK, params);
    
    % Coriolis (13x1, 위성 CoM 기준)
    % v_base: 위성 CoM 선속도 (관성좌표계)
    c = RA_C(FK, params, v_base(:), omega_inertial, theta_dot);
    
    % 일반화 힘
    Q = [zeros(6,1); tau(:)];
    
    % Forward Dynamics
    % zeta = [v_base; omega_inertial; theta_dot]
    zeta_ddot = H \ (Q - c);
    
    v_base_dot = zeta_ddot(1:3);
    omega_dot_inertial = zeta_ddot(4:6);
    theta_ddot = zeta_ddot(7:13);
    
    % 각가속도: inertial → body frame
    omega_dot_body = R_base' * omega_dot_inertial;
    
    % Quaternion 미분
    q_dot = Derivative_Quat(q_base, omega_base);
    
    % 상태 미분 벡터
    ds = [v_base(:); q_dot(:); v_base_dot(:); omega_dot_body(:); theta_dot(:); theta_ddot(:)];
end

%% 상태 패킹
function s = pack_state(p, q, v, omega, theta, theta_dot)
    s = [p(:); q(:); v(:); omega(:); theta(:); theta_dot(:)];
end

%% 상태 언패킹
function [p, q, v, omega, theta, theta_dot] = unpack_state(s)
    p = s(1:3);
    q = s(4:7);
    v = s(8:10);
    omega = s(11:13);
    theta = s(14:20);
    theta_dot = s(21:27);
end