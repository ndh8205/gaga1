%% measure_sensor_ST_simple.m
function [z_measured, z_true, ST_params] = measure_sensor_ST_simple(Xk, ~, ~, ST_params, dt)
% 단순화된 STAR TRACKER 측정 모델
% 
% 가정:
%   - ECI 기준 자세를 직접 측정: z = q_I^B
%   - White noise만 존재 (바이어스 없음)
%   - 보어사이트 정렬 완벽 (identity)
%
% Inputs:
%   Xk        - Deputy 상태벡터 [r_B^I(3); v_B^I(3); q_I^B(4); ...]
%   ST_params - ST 파라미터 (optional)
%   dt        - 샘플 간격 [s] (미사용, 인터페이스 유지)
%
% Outputs:
%   z_measured - 측정된 쿼터니언 q_I^B (4x1)
%   z_true     - 실제 쿼터니언 q_I^B (4x1)
%   ST_params  - 파라미터 (업데이트됨)

%% 파라미터 초기화
if nargin < 4 || isempty(ST_params)
    ST_params = initialize_ST_params_simple();
end

%% Ground Truth 추출
q_I2B = Xk(7:10);  % ECI to Body
q_true = q_I2B / norm(q_I2B);

%% 측정 노이즈 추가
% White noise: Δθ ~ N(0, σ²I)
sigma = ST_params.noise_std_rad;  % [rad]
delta_theta = sigma * randn(3,1);

% Small angle quaternion
dq_noise = [1; 0.5 * delta_theta];
dq_noise = dq_noise / norm(dq_noise);

% 노이즈 적용 (local multiplication)
q_meas = q2q_mult(q_true, dq_noise);
q_meas = q_meas / norm(q_meas);

%% 연속성 보장 (부호 선택)
if ST_params.initialized
    q_meas = EnsQuatCont(q_meas, ST_params.q_prev);
else
    ST_params.initialized = true;
end

ST_params.q_prev = q_meas;

%% 출력
z_measured = q_meas;  % q_I^B (measured)
z_true = q_true;      % q_I^B (true)

end


%% ========== 초기화 함수 ==========
function params = initialize_ST_params_simple()
% 단순화된 Star Tracker 파라미터

params.initialized = false;

% 측정 노이즈
noise_rms_arcsec = 20;
params.noise_std_rad = noise_rms_arcsec * (pi/180) / 3600;  % rad

% 내부 상태
params.q_prev = [1; 0; 0; 0];

end