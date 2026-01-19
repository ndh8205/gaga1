function c = RA_C_dual(FK_dual, params, v_base, omega_base, theta_dot_L, theta_dot_R)
% RA_C_dual: 양팔 시스템 Coriolis/Centrifugal 비선형 항 계산
%
% 기존 RA_C를 재사용하여 안정성 보장
%
% =========================================================================
% 동역학 방정식:
%   H * zeta_ddot + c = tau_gen
%
% c 구조 (20x1):
%   c = [c_b; c_m_L; c_m_R]
%       c_b   (6x1)  - Base 비선형 항
%       c_m_L (7x1)  - 왼팔 관절 비선형 항
%       c_m_R (7x1)  - 오른팔 관절 비선형 항
%
% 핵심:
%   - 기존 RA_C 함수 재사용
%   - 위성 관성 기여 1번만 (중복 제거)
%   - 양팔 기여 합산
%
% =========================================================================
% 입력 (Inputs):
%   FK_dual    - RA_FK_dual 함수의 출력 구조체
%   params     - 시스템 파라미터 구조체
%   v_base     - Base CoM 선속도 (3x1) [m/s], 관성좌표계
%   omega_base - Base 각속도 (3x1) [rad/s], 관성좌표계
%   theta_dot_L - 왼팔 관절 각속도 (7x1) [rad/s]
%   theta_dot_R - 오른팔 관절 각속도 (7x1) [rad/s]
%
% 출력 (Outputs):
%   c - 비선형 항 벡터 (20x1)
%
% =========================================================================

%% ========== 입력 검증 ==========
v_base = v_base(:);
omega_base = omega_base(:);
theta_dot_L = theta_dot_L(:);
theta_dot_R = theta_dot_R(:);

n_joints = params.arm.n_joints;  % 7

%% ========== 왼팔 Coriolis (기존 RA_C 호출) ==========
c_L = RA_C(FK_dual.L, params, v_base, omega_base, theta_dot_L);
% c_L = [f_L(3); n_L(3); tau_L(7)] = 13x1

%% ========== 오른팔 Coriolis (기존 RA_C 호출) ==========
c_R = RA_C(FK_dual.R, params, v_base, omega_base, theta_dot_R);
% c_R = [f_R(3); n_R(3); tau_R(7)] = 13x1

%% ========== 위성 Coriolis 기여 (중복 제거 필요) ==========
% RA_C는 위성 기여를 포함하므로, 위성 전용 기여분 계산
% 위성만 있을 때의 Coriolis = cross(omega, I_sat * omega)

I_sat_I = FK_dual.R_base * params.sat.I * FK_dual.R_base';
n_sat = cross(omega_base, I_sat_I * omega_base);

% RA_C 내부에서 위성 기여가 어떻게 처리되는지 확인 필요
% RA_C는 위성 + 로봇팔 전체를 계산
% 양팔을 합산하면 위성이 2번 카운트됨

%% ========== c_b 조립 (양팔 합산, 위성 중복 제거) ==========
% 각 RA_C 결과에서 위성 기여 제거 후 합산
% 또는: c_b = c_L(1:6) + c_R(1:6) - [0; n_sat]

% 방법: 위성 각운동량 Coriolis만 중복
% f_sat = 0 (선운동량은 외력 없으면 0)
% n_sat = cross(omega, I*omega)

c_b_f = c_L(1:3) + c_R(1:3);  % 힘: 단순 합산 (위성 선운동량 기여 0)
c_b_n = c_L(4:6) + c_R(4:6) - n_sat;  % 모멘트: 위성 중복 제거

c_b = [c_b_f; c_b_n];

%% ========== 관절 토크 ==========
tau_L = c_L(7:13);
tau_R = c_R(7:13);

%% ========== 출력 조립 (20x1) ==========
c = [c_b; tau_L; tau_R];

end