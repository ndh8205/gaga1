function [z_measured, z_true, bias_states, param] = measure_sensor_IMU(Xk, X_target, Rk, param, dt)
% IMU 센서 측정 모델 (LaTeX 문서 §5.6 컨벤션과 일치)
% IMU는 Deputy Body frame(B)에 장착. ECI(I) 자유낙하 가정(추력 없음).
%
% Inputs:
%   Xk        - Deputy 상태벡터 [r_B^I(3); v_B^I(3); q_I^B(4); omega_B/I^B(3)]
%   X_target  - Chief 상태벡터 (현재 미사용; 인터페이스 유지용)
%   Rk        - 측정 잡음 공분산 (현재 미사용; 인터페이스 유지용)
%   param     - 파라미터 구조체 (IMU 바이어스 상태 포함)
%   dt        - 샘플 간격 [s]
%
% Outputs:
%   z_measured  - 측정치 [a_f^M(3); omega_B/I^M(3)]
%   z_true      - 실제값 [a_f^B(3); omega_B/I^B(3)]
%   bias_states - 모든 바이어스 상태 [24x1]
%                 [b_a(3); eps_a(6); s_a(3); b_gy(3); eps_gy(6); s_gy(3)]
%   param       - 업데이트된 파라미터 구조체

% ---------- IMU 파라미터 초기화 (첫 호출 시) ----------
if ~isfield(param, 'imu') || ~isfield(param.imu, 'initialized')
    param = initialize_imu_params(param);
end

% ---------- 상태 분해 ----------
r_B_I = Xk(1:3);                % [km] Deputy position - ECI
v_B_I = Xk(4:6);                % [km/s] Deputy velocity - ECI
q_I2B = Xk(7:10);               % quaternion: ECI -> Body
q_I2B = q_I2B(:) / norm(q_I2B); % 정규화
omega_BI_B = Xk(11:13);         % [rad/s], ω^B_B/I

% ---------- 궤도 파라미터 ----------
Mu = param.orbit.Mu;    % [km³/s²]
R_e = param.orbit.R_e;   % [km]
J2 = param.orbit.J2;     % [-]

% ---------- 중력 및 J2 (ECI) ----------
r_norm = norm(r_B_I);
r3 = r_norm^3;
r5 = r_norm^5;

% J2 섭동을 위한 rho 행렬
rho_1 = 1 - 5 * (r_B_I(3)/r_norm)^2;
rho_2 = rho_1;
rho_3 = 3 - 5 * (r_B_I(3)/r_norm)^2;
rho_mat = diag([rho_1; rho_2; rho_3]);

% 중력 가속도 (ECI frame)
g_I = -Mu/r3 * r_B_I - 1.5 * J2 * Mu * (R_e^2/r5) * rho_mat * r_B_I;

% ---------- Specific Force 계산 ----------
% 자유낙하 가정: thrust = 0
% a_f^I = a_actual - g = (g + thrust) - g = 0
a_f_I = zeros(3,1);  % 자유낙하이므로 specific force = 0

% ECI에서 Body 좌표계로 변환
R_I2B = GetDCM_QUAT(q_I2B);
% R_I2B = R_B2I';
a_f_B_true = R_I2B * a_f_I;

% ---------- 바이어스 업데이트 (시간 전파) ----------
% FOGM (First-Order Gauss-Markov) 모델 사용
% 또는 RW (Random Walk) 모델 선택 가능

% 가속도계 바이어스 업데이트
for i = 1:3
    param.imu.b_a(i) = param.imu.b_a(i) + ...
        FOGM_bias(param.imu.b_a(i), param.imu.tau_a, param.imu.sigma_a) * dt;
end

% 가속도계 misalignment 업데이트
for i = 1:6
    param.imu.eps_a(i) = param.imu.eps_a(i) + ...
        FOGM_bias(param.imu.eps_a(i), param.imu.tau_eps_a, param.imu.sigma_eps_a) * dt;
end

% 가속도계 scale factor 업데이트
for i = 1:3
    param.imu.s_a(i) = param.imu.s_a(i) + ...
        FOGM_bias(param.imu.s_a(i), param.imu.tau_s_a, param.imu.sigma_s_a) * dt;
end

% 자이로 바이어스 업데이트
for i = 1:3
    param.imu.b_gy(i) = param.imu.b_gy(i) + ...
        FOGM_bias(param.imu.b_gy(i), param.imu.tau_gy, param.imu.sigma_gy) * dt;
end

% 자이로 misalignment 업데이트
for i = 1:6
    param.imu.eps_gy(i) = param.imu.eps_gy(i) + ...
        FOGM_bias(param.imu.eps_gy(i), param.imu.tau_eps_gy, param.imu.sigma_eps_gy) * dt;
end

% 자이로 scale factor 업데이트
for i = 1:3
    param.imu.s_gy(i) = param.imu.s_gy(i) + ...
        FOGM_bias(param.imu.s_gy(i), param.imu.tau_s_gy, param.imu.sigma_s_gy) * dt;
end

% ---------- 현재 바이어스 값 추출 ----------
b_a = param.imu.b_a;
eps_a = param.imu.eps_a;
s_a = param.imu.s_a;
b_gy = param.imu.b_gy;
eps_gy = param.imu.eps_gy;
s_gy = param.imu.s_gy;

% ---------- 오차 행렬 구성 ----------
% Non-orthogonality & misalignment of the axis [acc]
M_a = [     0,      -eps_a(2),  eps_a(1);
        eps_a(4),        0,     -eps_a(3);
       -eps_a(6),    eps_a(5),      0     ];

% Errors due to scale-factor uncertainties [acc]
S_a = diag(s_a);

% Non-orthogonality & misalignment of the axis [gyro]
M_gy = [     0,      -eps_gy(2),  eps_gy(1);
         eps_gy(4),       0,      -eps_gy(3);
        -eps_gy(6),   eps_gy(5),      0     ];

% Errors due to scale-factor uncertainties [gyro]
S_gy = diag(s_gy);

% ---------- 백색 노이즈 ----------
eta_a = param.imu.noise_a * randn(3,1) / sqrt(dt);      % acc noise [km/s²]
eta_gy = param.imu.noise_gy * randn(3,1) / sqrt(dt);    % gyro noise [rad/s]

% ---------- IMU 측정값 생성 ----------
% 가속도계 측정
a_f_M = (eye(3) + M_a) * (eye(3) + S_a) * (a_f_B_true + b_a + eta_a);

% 자이로 측정
omega_M = (eye(3) + M_gy) * (eye(3) + S_gy) * (omega_BI_B + b_gy + eta_gy);

% ---------- 바이어스 상태 벡터 구성 ----------
bias_states = [b_a;      % 3x1
               eps_a;    % 6x1
               s_a;      % 3x1
               b_gy;     % 3x1
               eps_gy;   % 6x1
               s_gy];    % 3x1
               % 총 24x1

% ---------- 출력 ----------
z_measured = [a_f_M; omega_M];
z_true = [a_f_B_true; omega_BI_B];
end

% ========================================================================
% IMU 파라미터 초기화 함수
% ========================================================================
function param = initialize_imu_params(param)
    % IMU 초기화 플래그
    param.imu.initialized = true;
    
    % ---------- 가속도계 파라미터 ----------
    % 초기 바이어스 값
    param.imu.b_a = 1e-6 * randn(3,1);        % [km/s²]
    param.imu.eps_a = 1e-3 * randn(6,1);      % misalignment [rad]
    param.imu.s_a = 1e-4 * randn(3,1);        % scale factor error [-]

    % FOGM 파라미터 (가속도계)
    param.imu.tau_a = 3600;         % 상관 시간 [s] (1시간)
    param.imu.sigma_a = 1e-9;       % 프로세스 노이즈 강도 [km/s²/√s]

    param.imu.tau_eps_a = 7200;     % misalignment 상관 시간 [s]
    param.imu.sigma_eps_a = 1e-6;   % misalignment 노이즈 강도 [rad/√s]

    param.imu.tau_s_a = 7200;       % scale factor 상관 시간 [s]
    param.imu.sigma_s_a = 1e-7;     % scale factor 노이즈 강도 [1/√s]

    % ---------- 자이로 파라미터 ----------
    % 초기 바이어스 값
    param.imu.b_gy = deg2rad(1e-4 * randn(3,1));    % [rad/s]
    param.imu.eps_gy = 5e-4 * randn(6,1);           % misalignment [rad]
    param.imu.s_gy = 5e-5 * randn(3,1);             % scale factor error [-]

    % FOGM 파라미터 (자이로)
    param.imu.tau_gy = 3600;            % 상관 시간 [s] (1시간)
    param.imu.sigma_gy = deg2rad(1e-5); % 프로세스 노이즈 강도 [rad/s/√s]

    param.imu.tau_eps_gy = 7200;        % misalignment 상관 시간 [s]
    param.imu.sigma_eps_gy = 5e-7;      % misalignment 노이즈 강도 [rad/√s]

    param.imu.tau_s_gy = 7200;          % scale factor 상관 시간 [s]
    param.imu.sigma_s_gy = 5e-8;        % scale factor 노이즈 강도 [1/√s]

    % ---------- 측정 백색 노이즈 ----------
    param.imu.noise_a = 1e-6;           % 가속도계 백색 노이즈 [km/s²]
    param.imu.noise_gy = deg2rad(1e-3); % 자이로 백색 노이즈 [rad/s]

    % % ---------- 가속도계 파라미터 ----------
    % % 초기 바이어스 값
    % param.imu.b_a = 0 * randn(3,1);        % [km/s²]
    % param.imu.eps_a = 0 * randn(6,1);      % misalignment [rad]
    % param.imu.s_a = 0 * randn(3,1);        % scale factor error [-]
    % 
    % % FOGM 파라미터 (가속도계)
    % param.imu.tau_a = 3600;         % 상관 시간 [s] (1시간)
    % param.imu.sigma_a = 0;       % 프로세스 노이즈 강도 [km/s²/√s]
    % 
    % param.imu.tau_eps_a = 7200;     % misalignment 상관 시간 [s]
    % param.imu.sigma_eps_a = 0;   % misalignment 노이즈 강도 [rad/√s]
    % 
    % param.imu.tau_s_a = 7200;       % scale factor 상관 시간 [s]
    % param.imu.sigma_s_a = 0;     % scale factor 노이즈 강도 [1/√s]
    % 
    % % ---------- 자이로 파라미터 ----------
    % % 초기 바이어스 값
    % param.imu.b_gy = deg2rad(0 * randn(3,1));    % [rad/s]
    % param.imu.eps_gy = 0 * randn(6,1);           % misalignment [rad]
    % param.imu.s_gy = 0 * randn(3,1);             % scale factor error [-]
    % 
    % % FOGM 파라미터 (자이로)
    % param.imu.tau_gy = 3600;            % 상관 시간 [s] (1시간)
    % param.imu.sigma_gy = deg2rad(0); % 프로세스 노이즈 강도 [rad/s/√s]
    % 
    % param.imu.tau_eps_gy = 7200;        % misalignment 상관 시간 [s]
    % param.imu.sigma_eps_gy = 0;      % misalignment 노이즈 강도 [rad/√s]
    % 
    % param.imu.tau_s_gy = 7200;          % scale factor 상관 시간 [s]
    % param.imu.sigma_s_gy = 0;        % scale factor 노이즈 강도 [1/√s]
    % 
    % % ---------- 측정 백색 노이즈 ----------
    % param.imu.noise_a = 0;           % 가속도계 백색 노이즈 [km/s²]
    % param.imu.noise_gy = deg2rad(0); % 자이로 백색 노이즈 [rad/s]
    % 
    % % Random Walk 모델을 사용하려면:
    % param.imu.use_rw = true;  % FOGM 대신 RW 사용
    % param.imu.sigma_rw_a = 1e-10;    % RW 노이즈 강도 [km/s²/√s]
    % param.imu.sigma_rw_gy = deg2rad(1e-8); % RW 노이즈 강도 [rad/s/√s]
end