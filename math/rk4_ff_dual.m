function [p_new, q_new, v_new, omega_new, theta_L_new, theta_R_new, ...
          theta_dot_L_new, theta_dot_R_new] = ...
    rk4_ff_dual(p_base, q_base, v_base, omega_base, ...
                theta_L, theta_R, theta_dot_L, theta_dot_R, ...
                tau_func, t, dt, params)
% rk4_ff_dual: 양팔 Free-floating 시스템 RK4 적분기
%
% =========================================================================
% 상태 벡터 (34 DOF):
%   q = [p_base(3); q_base(4); theta_L(7); theta_R(7)]  - 위치/자세
%   zeta = [v_base(3); omega_base(3); theta_dot_L(7); theta_dot_R(7)] - 속도
%
% 동역학:
%   H * zeta_dot + c = Q
%   zeta_dot = H \ (Q - c)
%
% =========================================================================
% 입력 (Inputs):
%   p_base, q_base     - 위성 위치/자세 (관성좌표계)
%   v_base, omega_base - 위성 선속도/각속도 (관성좌표계, body frame)
%   theta_L, theta_R   - 양팔 관절각
%   theta_dot_L, theta_dot_R - 양팔 관절각속도
%   tau_func           - 토크 함수 @(t) -> [tau_L(7); tau_R(7)]
%   t                  - 현재 시간
%   dt                 - 시간 간격
%   params             - 시스템 파라미터
%
% 출력 (Outputs):
%   업데이트된 상태 변수들
%
% =========================================================================

%% 상태 패킹
state = pack_state_dual(p_base, q_base, v_base, omega_base, ...
                        theta_L, theta_R, theta_dot_L, theta_dot_R);

%% k1
tau1 = tau_func(t);
k1 = state_derivative_dual(p_base, q_base, v_base, omega_base, ...
                           theta_L, theta_R, theta_dot_L, theta_dot_R, ...
                           tau1, params);

%% k2
s2 = state + 0.5*dt*k1;
[p2, q2, v2, w2, thL2, thR2, thdL2, thdR2] = unpack_state_dual(s2);
q2 = q2 / norm(q2);
tau2 = tau_func(t + 0.5*dt);
k2 = state_derivative_dual(p2, q2, v2, w2, thL2, thR2, thdL2, thdR2, tau2, params);

%% k3
s3 = state + 0.5*dt*k2;
[p3, q3, v3, w3, thL3, thR3, thdL3, thdR3] = unpack_state_dual(s3);
q3 = q3 / norm(q3);
tau3 = tau_func(t + 0.5*dt);
k3 = state_derivative_dual(p3, q3, v3, w3, thL3, thR3, thdL3, thdR3, tau3, params);

%% k4
s4 = state + dt*k3;
[p4, q4, v4, w4, thL4, thR4, thdL4, thdR4] = unpack_state_dual(s4);
q4 = q4 / norm(q4);
tau4 = tau_func(t + dt);
k4 = state_derivative_dual(p4, q4, v4, w4, thL4, thR4, thdL4, thdR4, tau4, params);

%% 상태 업데이트
state_new = state + (dt/6)*(k1 + 2*k2 + 2*k3 + k4);

%% 언패킹
[p_new, q_new, v_new, omega_new, theta_L_new, theta_R_new, ...
 theta_dot_L_new, theta_dot_R_new] = unpack_state_dual(state_new);
q_new = q_new / norm(q_new);

end


%% ========== 상태 미분 계산 ==========
function ds = state_derivative_dual(p_base, q_base, v_base, omega_base, ...
                                    theta_L, theta_R, theta_dot_L, theta_dot_R, ...
                                    tau, params)
% state_derivative_dual: 상태 미분 벡터 계산
%
% 입력 tau = [tau_L(7); tau_R(7)] (14x1)

n_joints = params.arm.n_joints;

%% 입력 정리
tau = tau(:);
tau_L = tau(1:n_joints);
tau_R = tau(n_joints+1:2*n_joints);

%% 회전행렬
R_base = GetDCM_QUAT(q_base);

%% omega_base: body frame → inertial frame
omega_inertial = R_base * omega_base(:);

%% FK
FK_dual = RA_FK_dual(params, p_base, q_base, theta_L, theta_R);

%% Mass Matrix (20x20)
H = RA_MM_dual(FK_dual, params);

%% Coriolis (20x1)
c = RA_C_dual(FK_dual, params, v_base(:), omega_inertial, theta_dot_L, theta_dot_R);

%% 일반화 힘 (20x1)
% Q = [F_ext(3); T_ext(3); tau_L(7); tau_R(7)]
% 외력 없음 (free-floating)
Q = [zeros(6, 1); tau_L; tau_R];

%% Forward Dynamics
% H * zeta_ddot = Q - c
zeta_ddot = H \ (Q - c);

v_base_dot = zeta_ddot(1:3);
omega_dot_inertial = zeta_ddot(4:6);
theta_ddot_L = zeta_ddot(7:13);
theta_ddot_R = zeta_ddot(14:20);

%% 각가속도: inertial → body frame
omega_dot_body = R_base' * omega_dot_inertial;

%% Quaternion 미분
q_dot = Derivative_Quat(q_base, omega_base);

%% 상태 미분 벡터 조립 (34x1)
ds = [v_base(:);           % 3: p_dot
      q_dot(:);            % 4: q_dot
      v_base_dot(:);       % 3: v_dot
      omega_dot_body(:);   % 3: omega_dot (body)
      theta_dot_L(:);      % 7: theta_L_dot
      theta_dot_R(:);      % 7: theta_R_dot
      theta_ddot_L(:);     % 7: theta_dot_L_dot
      theta_ddot_R(:)];    % 7: theta_dot_R_dot

end


%% ========== 상태 패킹 ==========
function s = pack_state_dual(p, q, v, omega, theta_L, theta_R, theta_dot_L, theta_dot_R)
    s = [p(:); q(:); v(:); omega(:); ...
         theta_L(:); theta_R(:); theta_dot_L(:); theta_dot_R(:)];
end


%% ========== 상태 언패킹 ==========
function [p, q, v, omega, theta_L, theta_R, theta_dot_L, theta_dot_R] = unpack_state_dual(s)
    p = s(1:3);
    q = s(4:7);
    v = s(8:10);
    omega = s(11:13);
    theta_L = s(14:20);
    theta_R = s(21:27);
    theta_dot_L = s(28:34);
    theta_dot_R = s(35:41);
end