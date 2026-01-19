function [P, L] = RA_Momentum(r_0, q_0, v_0, omega_0, theta, theta_dot, params)
% RA_Momentum: Free-floating 시스템 운동량 계산
%
% Inputs:
%   r_0       - 위성 CoM 위치 (3x1) [m]
%   q_0       - 위성 자세 quaternion (4x1) [qw; qx; qy; qz]
%   v_0       - 위성 CoM 선속도 (3x1) [m/s]
%   omega_0   - 위성 각속도 (3x1) [rad/s], body frame
%   theta     - 관절각 (7x1) [rad]
%   theta_dot - 관절각속도 (7x1) [rad/s]
%   params    - 시스템 파라미터
%
% Outputs:
%   P - 시스템 총 선운동량 (3x1) [kg*m/s], 관성계
%   L - 시스템 총 각운동량 (3x1) [kg*m^2/s], 관성계 원점 기준

%% FK 및 Jacobian
[T_list, p_com, R_list] = RA_FK(r_0, q_0, theta, params);
[Jv, Jw] = RA_Jacobian(r_0, q_0, theta, params);

%% 위성 회전행렬
R_0 = GetDCM_QUAT(q_0);
omega_0_inertial = R_0 * omega_0(:);

%% 위성 운동량
m_sat = params.sat.m;
I_sat_body = params.sat.I;
I_sat = R_0 * I_sat_body * R_0';

P = m_sat * v_0(:);
L = m_sat * cross(r_0(:), v_0(:)) + I_sat * omega_0_inertial;

%% 각 링크의 운동량 기여
n_links = params.arm.n_bodies;

for i = 1:n_links
    link = params.arm.links(i);
    m_i = link.m;
    I_i_local = link.I;  % CoM 기준
    R_i = R_list(:,:,i);
    I_i = R_i * I_i_local * R_i';
    
    % 링크 CoM 위치
    p_i = p_com(:,i);
    
    % 링크 속도 (위성 운동 + 관절 운동)
    Jv_i = Jv(:,:,i);
    Jw_i = Jw(:,:,i);
    
    % 위성 운동에 의한 링크 속도
    r_rel = p_i - r_0(:);  % 위성 CoM에서 링크 CoM까지
    v_i_sat = v_0(:) + cross(omega_0_inertial, r_rel);
    omega_i_sat = omega_0_inertial;
    
    % 관절 운동에 의한 링크 속도
    v_i_joint = Jv_i * theta_dot(:);
    omega_i_joint = Jw_i * theta_dot(:);
    
    % 총 링크 속도
    v_i = v_i_sat + v_i_joint;
    omega_i = omega_i_sat + omega_i_joint;
    
    % 선운동량 기여
    P = P + m_i * v_i;
    
    % 각운동량 기여 (관성계 원점 기준)
    L = L + m_i * cross(p_i, v_i) + I_i * omega_i;
end

end