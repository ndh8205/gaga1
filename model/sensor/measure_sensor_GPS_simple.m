%% measure_sensor_GPS_simple.m
function [z_measured, z_true, GPS_params] = measure_sensor_GPS_simple(Xk, ~, ~, GPS_params, dt)
% 단순화된 GPS 측정 모델 (큐브샛 사양)
% 
% 가정:
%   - ECI 기준 위치/속도를 직접 측정: z = [r^I; v^I]
%   - White noise만 존재 (바이어스 없음)
%   - Clock error 무시
%
% Inputs:
%   Xk         - Deputy 상태벡터 [r_B^I(3); v_B^I(3); q_I^B(4); ...]
%   GPS_params - GPS 파라미터 (optional)
%   dt         - 샘플 간격 [s] (미사용, 인터페이스 유지)
%
% Outputs:
%   z_measured - 측정된 [r^I(3); v^I(3)] [km, km/s]
%   z_true     - 실제 [r^I(3); v^I(3)] [km, km/s]
%   GPS_params - 파라미터 (업데이트됨)

%% 파라미터 초기화
if nargin < 4 || isempty(GPS_params)
    GPS_params = initialize_GPS_params_simple();
end

%% Ground Truth 추출
r_true = Xk(1:3);  % Position - ECI [km]
v_true = Xk(4:6);  % Velocity - ECI [km/s]

%% 측정 노이즈 추가
% 위치 노이즈 [km]
sigma_pos = GPS_params.noise_pos_std_km;
noise_pos = sigma_pos * randn(3,1);

% 속도 노이즈 [km/s]
sigma_vel = GPS_params.noise_vel_std_kms;
noise_vel = sigma_vel * randn(3,1);

%% 측정값 생성
r_meas = r_true + noise_pos;
v_meas = v_true + noise_vel;

%% 출력
z_measured = [r_meas; v_meas];  % 6x1
z_true = [r_true; v_true];      % 6x1

end


%% ========== 초기화 함수 ==========
function params = initialize_GPS_params_simple()
% 단순화된 GPS 파라미터 (큐브샛 일반 사양)

% === 노이즈 레벨 (1-sigma) ===
% 위치 정확도: 5-10m (일반적)
params.noise_pos_std_m = 7.5;  % [m] - 중간값
params.noise_pos_std_km = params.noise_pos_std_m / 1000;  % [km]

% 속도 정확도: 0.1-0.2 m/s (일반적)
params.noise_vel_std_ms = 0.15;  % [m/s] - 중간값
params.noise_vel_std_kms = params.noise_vel_std_ms / 1000;  % [km/s]

% === 메타 정보 ===
params.update_rate_hz = 1;  % 1Hz (일반적)
params.model_type = 'simple_white_noise';
params.frame = 'ECI';

% === 설명 ===
params.description = struct(...
    'manufacturer', 'Generic CubeSat GPS', ...
    'accuracy_pos_m', '7.5m (1σ)', ...
    'accuracy_vel_ms', '0.15 m/s (1σ)', ...
    'notes', 'White noise only, no bias/drift');

end