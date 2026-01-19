function [z_measured, z_true, camera_params] = measure_sensor_camera_v2_2(Xk, X_target, points_chief_body, camera_params, orbit_normal_vec, dt)
% 카메라 센서 측정 모델 (q_B2A 직접 사용 버전)
% 카메라는 Deputy Body frame(B)에 장착
%
% Inputs:
%   Xk                - Deputy 상태벡터 [r_B^I(3); v_B^I(3); q_I^B(4); omega_B/I^B(3)]
%   X_target          - Chief 상태벡터 [r_A^I(3); v_A^I(3); q_I^A(4); omega_A/I^A(3)]
%   points_chief_body - Chief body frame의 3D 포인트 (3xN) [km]
%   camera_params     - 카메라 파라미터 구조체 (없으면 내부에서 초기화)
%   orbit_normal_vec  - 궤도 평면 법선 벡터 (3x1)
%   dt                - 샘플 간격 [s]
%
% Outputs:
%   z_measured    - 측정된 픽셀 좌표와 가시성 (2xN + 1xN)
%   z_true        - 이상적인 픽셀 좌표와 가시성
%   camera_params - 업데이트된 카메라 파라미터


% ---------- 파라미터 초기화 ----------
if nargin < 4 || isempty(camera_params)
    camera_params = initialize_camera_params();
end

% ---------- 상태 분해 ----------
% Deputy 상태
r_B_I = Xk(1:3);        % [km]
v_B_I = Xk(4:6);        % [km/s]
q_I2B = Xk(7:10);       % quaternion: I -> B
q_I2B = q_I2B(:) / norm(q_I2B);
omega_BI_B = Xk(11:13); % [rad/s]

% Chief 상태
r_A_I = X_target(1:3);
v_A_I = X_target(4:6);
q_I2A = X_target(7:10);
q_I2A = q_I2A(:) / norm(q_I2A);
omega_AI_A = X_target(11:13);

% ---------- LVLH 프레임 정의 (Chief 중심) ----------
r_hat = r_A_I / norm(r_A_I);
h_hat = cross(r_A_I, v_A_I) / norm(cross(r_A_I, v_A_I));
t_hat = cross(h_hat, r_hat);
DCM_L2I = [r_hat, t_hat, h_hat];
DCM_I2L = DCM_L2I';

% ---------- 상대 위치 (LVLH) ----------
r_rel_I = r_B_I - r_A_I;  % Deputy relative to Chief in ECI
r_rel_L = DCM_I2L * r_rel_I;  % in LVLH

% ---------- 쿼터니언 계산 ----------
% I -> L 쿼터니언
q_I2L = DCM2Quat(DCM_I2L);

% L -> B (Deputy body) 쿼터니언  
q_L2B = q2q_mult(q_I2B, inv_q(q_I2L));

% B -> A (상대 자세)
q_B2A = q2q_mult(q_I2A, inv_q(q_I2B));
q_B2A = q_B2A / norm(q_B2A);

% ---------- Chief body → Camera 직접 변환 ----------
% Step 1: Chief body → Deputy body
R_B2A = GetDCM_QUAT(q_B2A);
R_A2B = R_B2A';  % A → B

% Chief 원점의 Deputy body frame 좌표
DCM_L2B = GetDCM_QUAT(q_L2B);
t_A_in_B = -DCM_L2B * r_rel_L;  % Chief 원점 위치 (Deputy body 좌표)

% Step 2: Deputy Body → Camera (장착 관계)
R_B2C = [1, 0, 0;    % X_cam = X_body
         0, 0, -1;    % Y_cam = Z_body
         0, 1, 0];   % Z_cam = Y_body

% Step 3: 합성
R_A2C = R_B2C * R_A2B;
t_A_in_C = R_B2C * t_A_in_B;

% ---------- 이상적인 투영 (오차 없음) ----------
num_points = size(points_chief_body, 2);
K_ideal = camera_params.intrinsic.K_ideal;
image_width = camera_params.intrinsic.image_width;
image_height = camera_params.intrinsic.image_height;

pixels_true = zeros(2, num_points);
visible_true = false(1, num_points);
depths = zeros(1, num_points);  % Depth 저장

for i = 1:num_points
    % Chief body → Camera 직접 변환
    p_cam = R_A2C * points_chief_body(:, i) + t_A_in_C;
    
    % 투영 (z > 0 체크)
    if p_cam(3) > 0
        u = K_ideal(1,1) * p_cam(1) / p_cam(3) + K_ideal(1,3);
        v = K_ideal(2,2) * p_cam(2) / p_cam(3) + K_ideal(2,3);
        pixels_true(:, i) = [u; v];
        depths(i) = p_cam(3);  % Depth 기록
        
        % FOV 체크
        if u >= 0 && u <= image_width && v >= 0 && v <= image_height
            visible_true(i) = true;
        end
    end
end
% ---------- Occlusion 처리 (디버깅 버전) ----------
% 디버깅 플래그
debug_occlusion = true;  % true로 설정하면 디버깅 정보 출력

if debug_occlusion
    fprintf('Current distance: %.3f km\n', norm(r_rel_L));
    fprintf('Visible points before occlusion: %d\n', sum(visible_true));
end

% ---------- 포인트 간격 계산 (초기 1회만) ----------
% persistent로 저장해서 재사용
persistent mean_point_spacing_cached;

if isempty(mean_point_spacing_cached)
    % 100개 포인트에서 각각 5개 최근접 이웃 검사
    num_samples = 100;
    k_neighbors = 5;
    all_spacings = [];
    
    % 균등하게 분포된 샘플 선택
    sample_idx = round(linspace(1, num_points, num_samples));
    
    for i = 1:num_samples
        idx = sample_idx(i);
        % idx번째 포인트와 모든 다른 포인트 간 거리
        distances_3d = vecnorm(points_chief_body(:,idx) - points_chief_body, 2, 1);
        distances_3d(idx) = inf;  % 자기 자신 제외
        
        % k개 최근접 이웃 거리
        sorted_dist = sort(distances_3d);
        nearest_k = sorted_dist(1:min(k_neighbors, length(sorted_dist)-1));
        all_spacings = [all_spacings, nearest_k];
    end
    
    mean_point_spacing_cached = mean(all_spacings(~isinf(all_spacings)));  % [km]
    
    if debug_occlusion
        fprintf('포인트 간격 계산 완료: %.1f mm (캐시됨)\n', mean_point_spacing_cached*1e6);
    end
end

% 캐시된 값 사용
mean_point_spacing = mean_point_spacing_cached;

% ---------- 거리 적응형 파라미터 ----------
current_distance = norm(r_rel_L);  % [km]

% 거리별 계수 조정 (가까울수록 보수적)
if current_distance < 0.1  % 100m 이내
    margin_factor = 0.5;  % 매우 보수적
    depth_multiplier = 1.0;
    max_pixel_limit = 5.0;  % 엄격한 상한
elseif current_distance < 0.3  % 300m 이내  
    margin_factor = 0.8;
    depth_multiplier = 1.5;
    max_pixel_limit = 10.0;
else  % 300m 이상
    margin_factor = 1.0;
    depth_multiplier = 2.0;
    max_pixel_limit = 15.0;
end

% FOV와 이미지 크기로 실제 픽셀/라디안 계산
fov_rad = camera_params.intrinsic.fov;
image_width = camera_params.intrinsic.image_width;
pixels_per_radian = image_width / fov_rad;

% 1픽셀이 커버하는 실제 거리 계산
pixel_angular_size = fov_rad / image_width;  % 1픽셀의 각크기 [rad]
pixel_physical_size = current_distance * tan(pixel_angular_size);  % [km]

% Adaptive min: 거리에 따라 조정
min_pixels_per_spacing = pixel_physical_size / mean_point_spacing;
adaptive_min = max(1.5, min_pixels_per_spacing * 0.8);  % 최소 1.5픽셀

% Adaptive max: 위성 크기와 거리별 제한 고려
satellite_size = 0.005;  % 5m 본체 두께 [km]
satellite_angular_size = atan(satellite_size / current_distance);
adaptive_max = min(satellite_angular_size * pixels_per_radian * 0.3, max_pixel_limit);

% ---------- Occlusion 파라미터 계산 ----------
% 평균 간격의 각크기를 픽셀로 변환 (거리별 계수 적용)
angular_spacing = atan(mean_point_spacing / current_distance);
pixel_radius = angular_spacing * pixels_per_radian * margin_factor;
pixel_radius = min(max(pixel_radius, adaptive_min), adaptive_max);

% ---------- Adaptive Depth Threshold 계산 ----------
% 거리 범위 제한 (너무 가깝거나 멀 때 보정)
normalized_distance = min(max(current_distance, 0.05), 1.0);  % 50m~1km 범위

% 거리에 따른 depth 해상도 (선형)
base_depth_resolution = 0.0001;  % 100m에서 0.1m
distance_factor = normalized_distance / 0.1;  % 100m 기준

% 카메라의 depth 정확도 (선형 증가)
depth_uncertainty = base_depth_resolution * distance_factor;

% 포인트 간격과 depth 불확실성 중 큰 값 (배열로 묶기)
adaptive_depth_threshold = max([...
    mean_point_spacing * depth_multiplier, ...  % 거리별 계수 적용
    depth_uncertainty * 3, ...                  % depth 불확실성의 3배
    0.00005]);                                   % 최소 5cm

% 위성 구조 고려
if current_distance < 0.1  % 100m 이내
    satellite_thickness = 0.0005;  % 0.5m (엄격)
else
    satellite_thickness = 0.001;  % 1m
end
depth_threshold = min(adaptive_depth_threshold, satellite_thickness);

if debug_occlusion
    fprintf('Distance factor: margin=%.2f, depth_mult=%.1f, max_px=%.1f\n', ...
            margin_factor, depth_multiplier, max_pixel_limit);
    fprintf('1px covers: %.1f mm, Adaptive range: [%.1f ~ %.1f] px\n', ...
            pixel_physical_size*1e6, adaptive_min, adaptive_max);
    fprintf('Pixel radius: %.2f px, Depth thresh: %.1f mm\n', ...
            pixel_radius, depth_threshold*1000);
end

% ---------- Occlusion 검사 ----------
occluded_count = 0;
for i = 1:num_points
    if ~visible_true(i)
        continue;
    end
    
    u_i = pixels_true(1, i);
    v_i = pixels_true(2, i);
    d_i = depths(i);
    
    for j = 1:num_points
        if i == j || ~visible_true(j)
            continue;
        end
        
        u_j = pixels_true(1, j);
        v_j = pixels_true(2, j);
        d_j = depths(j);
        
        pixel_dist = sqrt((u_i - u_j)^2 + (v_i - v_j)^2);
        
        if pixel_dist < pixel_radius && d_j < d_i - depth_threshold
            visible_true(i) = false;
            occluded_count = occluded_count + 1;
            break;
        end
    end
end

if debug_occlusion
    fprintf('Occluded: %d, Visible: %d\n\n', occluded_count, sum(visible_true));
end

% ---------- 센서 오차 모델 ----------
% 캘리브레이션 오차가 적용된 카메라 행렬
K_measured = K_ideal;
K_measured(1,1) = K_ideal(1,1) * (1 + camera_params.calib.fx_error);
K_measured(2,2) = K_ideal(2,2) * (1 + camera_params.calib.fy_error);
K_measured(1,3) = K_ideal(1,3) + camera_params.calib.cx_error;
K_measured(2,3) = K_ideal(2,3) + camera_params.calib.cy_error;

% 렌즈 왜곡 계수
k1 = camera_params.distortion.k1;
k2 = camera_params.distortion.k2;
p1 = camera_params.distortion.p1;
p2 = camera_params.distortion.p2;

pixels_measured = zeros(2, num_points);
visible_measured = false(1, num_points);

for i = 1:num_points
    if visible_true(i)
        % 정규화 좌표 계산
        u_true = pixels_true(1, i);
        v_true = pixels_true(2, i);
        cx = K_ideal(1, 3);
        cy = K_ideal(2, 3);
        fx = K_ideal(1, 1);
        fy = K_ideal(2, 2);
        
        x_norm = (u_true - cx) / fx;
        y_norm = (v_true - cy) / fy;
        r2 = x_norm^2 + y_norm^2;
        
        % 왜곡 모델 적용
        radial_factor = 1 + k1*r2 + k2*r2^2;
        x_dist = x_norm * radial_factor + 2*p1*x_norm*y_norm + p2*(r2 + 2*x_norm^2);
        y_dist = y_norm * radial_factor + p1*(r2 + 2*y_norm^2) + 2*p2*x_norm*y_norm;
        
        % 왜곡된 픽셀 좌표 (측정된 K 사용)
        u_meas = K_measured(1,1) * x_dist + K_measured(1,3);
        v_meas = K_measured(2,2) * y_dist + K_measured(2,3);
        
        % 픽셀 노이즈 추가
        noise = camera_params.noise.pixel_std * randn(2, 1);
        pixels_measured(:, i) = [u_meas; v_meas] + noise;
        
        % 측정 가시성 체크 (노이즈로 인해 경계 밖으로 나갈 수 있음)
        if pixels_measured(1, i) >= -10 && pixels_measured(1, i) <= image_width+10 && ...
           pixels_measured(2, i) >= -10 && pixels_measured(2, i) <= image_height+10
            visible_measured(i) = true;
        else
            visible_measured(i) = false;
        end
    end
end

% ---------- 파라미터 동역학 업데이트 (Gauss-Markov 프로세스) ----------
% 캘리브레이션 오차 업데이트
tau_calib = camera_params.calib.drift_time_constant;
sigma_calib = sqrt(camera_params.calib.drift_noise_density);

camera_params.calib.fx_error = exp(-dt/tau_calib) * camera_params.calib.fx_error + ...
    sigma_calib * randn * sqrt(dt);
camera_params.calib.fy_error = exp(-dt/tau_calib) * camera_params.calib.fy_error + ...
    sigma_calib * randn * sqrt(dt);
camera_params.calib.cx_error = exp(-dt/tau_calib) * camera_params.calib.cx_error + ...
    sigma_calib * randn * sqrt(dt) * 10;  % pixels scale
camera_params.calib.cy_error = exp(-dt/tau_calib) * camera_params.calib.cy_error + ...
    sigma_calib * randn * sqrt(dt) * 10;

% 렌즈 왜곡 계수 업데이트
tau_dist = camera_params.distortion.drift_time_constant;
sigma_dist = sqrt(camera_params.distortion.drift_noise_density);

camera_params.distortion.k1 = camera_params.distortion.k1 + ...
    sigma_dist * randn * sqrt(dt);
camera_params.distortion.k2 = camera_params.distortion.k2 + ...
    sigma_dist * randn * sqrt(dt) * 0.1;  % k2는 더 작게

% ---------- 출력 구조체 생성 ----------
z_true = [pixels_true; visible_true];  % (2+1) x N
z_measured = [pixels_measured; visible_measured];  % (2+1) x N

end

% ========================================================================
% 카메라 파라미터 초기화
% ========================================================================
function params = initialize_camera_params()
    deg = pi/180;
    
    % -------- Intrinsic Parameters --------
    params.intrinsic.image_width = 1024;
    params.intrinsic.image_height = 1024;
    params.intrinsic.fov = 3.6 * deg;  % [rad]
    
    % % -------- Intrinsic Parameters --------
    % params.intrinsic.image_width = 1024;
    % params.intrinsic.image_height = 1024;
    % params.intrinsic.fov = 0.1;  % [rad]

    % params.intrinsic.image_width = 1024;
    % params.intrinsic.image_height = 1024;
    % params.intrinsic.fov = 20 * deg;  % [rad]
    
    % 이상적인 카메라 행렬
    f = params.intrinsic.image_width / (2 * tan(params.intrinsic.fov/2));
    params.intrinsic.K_ideal = [f, 0, params.intrinsic.image_width/2;
                                 0, f, params.intrinsic.image_height/2;
                                 0, 0, 1];
    
    % -------- Calibration Errors --------
    params.calib.fx_error = 1e-3 * randn;
    params.calib.fy_error = 1e-3 * randn;
    params.calib.cx_error = 1.0 * randn;
    params.calib.cy_error = 1.0 * randn;
    
    % 캘리브레이션 드리프트 파라미터
    params.calib.drift_time_constant = 86400;  % 1일 [s]
    params.calib.drift_noise_density = 1e-12;
    
    % -------- Lens Distortion --------
    params.distortion.k1 = 1e-7;
    params.distortion.k2 = 1e-10;
    params.distortion.p1 = 1e-8;
    params.distortion.p2 = 1e-8;
    
    params.distortion.drift_time_constant = 864000;  % 10일 [s]
    params.distortion.drift_noise_density = 1e-18;
    
    % -------- Measurement Noise --------
    params.noise.pixel_std = 0.5;  % [pixels]
    
    % -------- Geometry --------
    params.geom.r_BC_B = [0; 0; 0];  % body 중심에 위치 [km]
    params.geom.q_B_C = [1; 0; 0; 0];
end