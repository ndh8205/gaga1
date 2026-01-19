clc; clear all; close all;
addpath(genpath('D:\star_tracker_test\main_pj_code'));

%% Initialise the gravitational constant and planet radius.
Mu = 3.98600441500000e+05; % Earth gravitational parameter [km^3/s^2]
R_e = 6.37813630000000e+03; % Earth radius [km]
J2 = 0.001082627; % Second zonal harmonic coefficient [-]

param.orbit.Mu = Mu;
param.orbit.R_e = R_e;
param.orbit.J2 = J2;

%% Chief [A] satellite orbital elements
e_A = 0; % Eccentricity [-]
a_A = 6908.136; % Semi-major axis [km] 530 km 고도
h_A = sqrt( a_A*( 1 - e_A^2 ) * Mu ); % Specific angular momentum [km^2/s]
i_A = deg2rad( 97.45 ); % Inclination [rad] - 고정
RAAN_A = deg2rad( 270.8 ); % Right ascension of ascending node [rad]
omega_A = deg2rad( 90 ); % Argument of periapsis [rad] - 고정
theta_A = deg2rad( 0 ); % True anomaly [rad]

[r_A, v_A] = coe2rv( [ h_A, e_A, RAAN_A, i_A, omega_A, theta_A ], Mu );
x_A = [ r_A; v_A ];

%% Deputy [B] - LVLH frame initial value - GCO formation
n = sqrt( Mu/a_A^3 ); % Mean motion [rad/s]
T_Op = 2 * pi/n; % Orbital period [s]
w_e = 7.29*10^(-5);

% GCO initial LVLH state
p_pi_B = deg2rad( 0 );
r_init_B = 1.2;

x_0_B = r_init_B/2 * sin( p_pi_B );
x_0_dot_B = r_init_B * n/2 * cos( p_pi_B );
y_0_B = 2 * x_0_dot_B/n;
y_0_dot_B = -2 * n * x_0_B;
z_0_B = sqrt(3) * x_0_B;
z_0_dot_B = sqrt(3) * x_0_dot_B;

r_0_B = [ x_0_B; y_0_B; z_0_B ];
v_0_B = [ x_0_dot_B; y_0_dot_B; z_0_dot_B ];

% % In Track initial LVLH state
% dist_formation = 0.5; %LVLH [km]
% 
% x_0_B = 0; % Initial x position LVLH [km]
% x_0_dot_B = 0; % Initial x velocity LVLH [km/s]
% y_0_B = 0 - dist_formation; % Initial y position LVLH [km]
% y_0_dot_B = 0; % Initial y velocity LVLH [km/s]
% z_0_B = 0; % Initial z position LVLH [km]
% z_0_dot_B = 0; % Initial z velocity LVLH [km/s]
% 
% r_0_B = [ x_0_B; y_0_B; z_0_B ]; % Initial position vector LVLH [km]
% v_0_B = [ x_0_dot_B; y_0_dot_B; z_0_dot_B ]; % Initial velocity vector LVLH [km/s]

%% ECI로 좌표변환 - GCO 위성
hB_I = cross( r_A, v_A );

i_I = r_A/norm( r_A );
k_I = hB_I/norm( hB_I );
j_I = cross( k_I, i_I );

R_L2I = [ i_I, j_I, k_I ];
R_I2L = R_L2I';

w_r_B = [ -n * r_0_B(2); n * r_0_B(1); 0 ];
r_B = r_A + ( R_L2I * r_0_B );
v_B = v_A + ( R_L2I * ( v_0_B + w_r_B ) );

x_B = [ r_B; v_B ];

%% LQR Controller Design
fprintf('\n=== LQR 제어기 설계 ===\n');

% CW state-space (LVLH frame)
A_cw = [zeros(3), eye(3);
        diag([3*n^2, 0, -n^2]), [0, 2*n, 0; -2*n, 0, 0; 0, 0, 0]];
B_cw = [zeros(3); eye(3)];

% LQR weights
q_pos = 1.0;    % 위치 가중치
q_vel = 1.0;    % 속도 가중치  
r_u = 1.0;      % 제어 입력 가중치

Q = 1 * diag([q_pos, q_pos, q_pos, q_vel, q_vel, q_vel]);
R = 1e+12 * diag([r_u, r_u, r_u]);

% LQR gain
K = lqr(A_cw, B_cw, Q, R);

fprintf('LQR gain 계산 완료\n');
fprintf('  Q: pos=%.1f, vel=%.1f\n', q_pos, q_vel);
fprintf('  R: %.2f\n', r_u);

% Transition time
t_transition = 1 * T_Op;
fprintf('전환 시점: %.1f초 (%.2f 궤도)\n', t_transition, t_transition/T_Op);

%% Dynamics wrapper for RK4
orbit_dyn = @(x, u, params, ~) orbit_propagation_j2(0, x, u, [0;0;0], params);

%% Main dynamics loop (discrete control)
dtdtdt = 0.2;
% tt = 60 * 30 * 1;
tt = 3 * T_Op;
dt = dtdtdt;

% Time settings
camera_interval = 60;
camera_times_planned = (camera_interval:camera_interval:tt)';
fprintf('카메라 측정 계획: %d회 (%.0f초 간격)\n', length(camera_times_planned), camera_interval);

gps_interval = 1;
gps_times_planned = (gps_interval:gps_interval:tt)';
fprintf('GPS 측정 계획: %d회 (%.0f초 간격)\n', length(gps_times_planned), gps_interval);

camera_count = 0;
processed_camera_idx = [];
gps_count = 0;
processed_gps_idx = [];

tspan = 0 : dt : tt;
N_steps = length(tspan);

% Preallocate arrays
r_A_I = zeros(3, N_steps);
v_A_I = zeros(3, N_steps);
r_B_I = zeros(3, N_steps);
v_B_I = zeros(3, N_steps);
r_B_L = zeros(3, N_steps);
v_B_L = zeros(3, N_steps);

% Control history
control_history = struct();
control_history.u_I = zeros(3, N_steps);
control_history.u_L = zeros(3, N_steps);
control_history.thrust_mag = zeros(1, N_steps);

% Initial conditions
r_A_I(:,1) = x_A(1:3);
v_A_I(:,1) = x_A(4:6);
r_B_I(:,1) = x_B(1:3);
v_B_I(:,1) = x_B(4:6);

% Initial LVLH
r_hat = r_A_I(:,1)/norm(r_A_I(:,1));
rcv = cross(r_A_I(:,1), v_A_I(:,1));
h_hat = rcv/norm(rcv);
t_hat = cross(h_hat, r_hat);
R_L2I_0 = [r_hat, t_hat, h_hat];
R_I2L_0 = R_L2I_0';

del_r = r_B_I(:,1) - r_A_I(:,1);
r_B_L(:,1) = R_I2L_0 * del_r;
del_v = v_B_I(:,1) - v_A_I(:,1);
omega_0 = cross(r_A_I(:,1), v_A_I(:,1)) / (norm(r_A_I(:,1))^2);
v_B_L(:,1) = R_I2L_0 * (del_v - cross(omega_0, del_r));

target_X_init = [0; -0.15; 0; 0; 0; 0];  % In-track target

%% Orbit normal vector calculation
fprintf('\n=== 궤도 계산 시작 ===\n');
fprintf('시뮬레이션 진행 중 (%.1f Hz 간격)...\n', 1/dtdtdt);

% Propagation loop
for k = 1:N_steps-1
    t_current = tspan(k);
    
    % Current state
    x_A_current = [r_A_I(:,k); v_A_I(:,k)];
    x_B_current = [r_B_I(:,k); v_B_I(:,k)];
    
    % Current LVLH frame
    r_hat = r_A_I(:,k)/norm(r_A_I(:,k));
    rcv = cross(r_A_I(:,k), v_A_I(:,k));
    h_hat = rcv/norm(rcv);
    t_hat = cross(h_hat, r_hat);
    R_L2I_k = [r_hat, t_hat, h_hat];
    R_I2L_k = R_L2I_k';
    
    % Control calculation
    if t_current >= t_transition
        x_rel_L = [r_B_L(:,k); v_B_L(:,k)];
        x_target_t = target_X_init;
        u_L = -K * (x_rel_L - x_target_t);
        u_I = R_L2I_k * u_L;
    else
        u_L = [0; 0; 0];
        u_I = [0; 0; 0];
    end
    
    % Store control
    control_history.u_I(:,k) = u_I;
    control_history.u_L(:,k) = u_L;
    control_history.thrust_mag(k) = norm(u_I);
    
    % Propagate (RK4)
    w_I = [0; 0; 0];
    x_A_next = rk4(orbit_dyn, x_A_current, [0;0;0], param, dt);
    x_B_next = rk4(orbit_dyn, x_B_current, u_I, param, dt);
    
    % Store
    r_A_I(:,k+1) = x_A_next(1:3);
    v_A_I(:,k+1) = x_A_next(4:6);
    r_B_I(:,k+1) = x_B_next(1:3);
    v_B_I(:,k+1) = x_B_next(4:6);
    
    % LVLH conversion
    r_hat_next = r_A_I(:,k+1)/norm(r_A_I(:,k+1));
    rcv_next = cross(r_A_I(:,k+1), v_A_I(:,k+1));
    h_hat_next = rcv_next/norm(rcv_next);
    t_hat_next = cross(h_hat_next, r_hat_next);
    R_L2I_next = [r_hat_next, t_hat_next, h_hat_next];
    R_I2L_next = R_L2I_next';
    
    del_r_next = r_B_I(:,k+1) - r_A_I(:,k+1);
    r_B_L(:,k+1) = R_I2L_next * del_r_next;
    
    del_v_next = v_B_I(:,k+1) - v_A_I(:,k+1);
    omega_next = cross(r_A_I(:,k+1), v_A_I(:,k+1)) / (norm(r_A_I(:,k+1))^2);
    v_B_L(:,k+1) = R_I2L_next * (del_v_next - cross(omega_next, del_r_next));
    
    if mod(k, 1000) == 0
        fprintf('  진행: %d/%d (%.1f%%, t=%.1fs)\n', k, N_steps, k/N_steps*100, t_current);
    end
end

fprintf('궤도 계산 완료.\n');

LVLH_pos = r_B_L;
LVLH_vel = v_B_L;
t1 = tspan';

%% 2. 궤도 평면 법선 벡터 계산
[coeff, ~, ~] = pca(LVLH_pos');
orbit_normal_vec = coeff(:,3);

if orbit_normal_vec(3) < 0 
    orbit_normal_vec = -orbit_normal_vec; 
end

Rk_IMU = eye(6);
Rk_ST = eye(4);

%% 3. 카메라 파라미터 초기화 및 Point Cloud 로드
fprintf('카메라 파라미터 초기화 중...\n');
camera_params = [];
ST_params = [];
fprintf('Star Tracker 파라미터 초기화 중 (단순 모델)...\n');
GPS_params = [];
fprintf('GPS 파라미터 초기화 중 (단순 모델)...\n');

% mat_filename = 'chief_pointcloud.mat';
mat_filename = 'ksas_pc.mat';


if exist(mat_filename, 'file')
    fprintf('Point Cloud 불러오는 중: %s\n', mat_filename);
    load(mat_filename);
    
    fprintf('=== 불러온 Point Cloud 정보 ===\n');
    fprintf('총 포인트 개수: %d\n', ptCloud.Count);
    
    all_points_km = points' / 1000;
    total_points = size(all_points_km, 2);
    
    max_display_points = 500;
    
    if total_points > max_display_points
        fprintf('포인트 다운샘플링: %d -> %d\n', total_points, max_display_points);
        step = floor(total_points / max_display_points);
        point_idx = 1:step:total_points;
        point_idx = point_idx(1:min(max_display_points, length(point_idx)));
    else
        point_idx = 1:total_points;
    end
    
    num_points = length(point_idx);
    initial_pointcloud = all_points_km(:, point_idx);
    fprintf('실제 표시할 포인트 수: %d\n', num_points);
    
    cloud_center = mean(initial_pointcloud, 2);
    initial_pointcloud = initial_pointcloud - cloud_center;
    fprintf('포인트 클라우드 중심 보정: [%.3f, %.3f, %.3f] km\n', cloud_center);
else
    fprintf('경고: Point Cloud 파일이 없습니다. 기본 포인트 생성...\n');
    [X,Y,Z] = meshgrid(linspace(-0.05, 0.05, 5));
    initial_pointcloud = [X(:)'; Y(:)'; Z(:)'];
    num_points = size(initial_pointcloud, 2);
end

%% 위성 크기 분석
fprintf('\n=== 위성 크기 분석 ===\n');

points_analysis = all_points_km;

min_coords = min(points_analysis, [], 2);
max_coords = max(points_analysis, [], 2);
bbox_size = max_coords - min_coords;

fprintf('Bounding Box:\n');
fprintf('  X축: %.3f m (%.3f ~ %.3f km)\n', bbox_size(1)*1000, min_coords(1), max_coords(1));
fprintf('  Y축: %.3f m (%.3f ~ %.3f km)\n', bbox_size(2)*1000, min_coords(2), max_coords(2));
fprintf('  Z축: %.3f m (%.3f ~ %.3f km)\n', bbox_size(3)*1000, min_coords(3), max_coords(3));
fprintf('  대각선 길이: %.3f m\n', norm(bbox_size)*1000);

[coeff, ~, latent] = pca(points_analysis');
principal_lengths = 2 * sqrt(latent);

fprintf('\nPrincipal Axes (95%% 범위):\n');
fprintf('  주축 1: %.3f m\n', principal_lengths(1)*1000);
fprintf('  주축 2: %.3f m\n', principal_lengths(2)*1000);
fprintf('  주축 3: %.3f m\n', principal_lengths(3)*1000);

aspect_ratios = bbox_size / min(bbox_size);
fprintf('\n형상 비율 (최소축 대비):\n');
fprintf('  X:Y:Z = %.2f:%.2f:%.2f\n', aspect_ratios(1), aspect_ratios(2), aspect_ratios(3));

if max(aspect_ratios) > 3
    fprintf('\n⚠️ 비정형 구조 감지 (태양패널 추정)\n');
    fprintf('  최대/최소 축 비율: %.1f\n', max(aspect_ratios));
end

%% 지구 텍스처 로드
obj_folder = 'C:\Users\USER\Desktop\relative2\m3d_src\earth_obj\';
tex_path = [obj_folder 'texture1.jpg'];
if exist(tex_path, 'file')
    earth_texture = imread(tex_path);
    earth_texture = imresize(earth_texture, 0.5);
    earth_texture = flipud(earth_texture);
    fprintf('지구 텍스처 로드 완료: %s\n', tex_path);
else
    error('지구 텍스처 파일을 찾을 수 없습니다: %s', tex_path);
end

omega_earth_dps = 360 / 86400;

%% 4. 실시간 시뮬레이션 및 센서 측정
fprintf('실시간 센서 측정 시뮬레이션 시작...\n');
fprintf('  - IMU/ST: %.1f Hz (%d 측정 예상)\n', 1/dtdtdt, length(t1));
fprintf('  - GPS: 1 Hz (~%d 측정 예상)\n', floor(tt/gps_interval));
fprintf('  - Camera: 1분 간격 (~%d 측정 예상)\n', floor(tt/camera_interval));

rotation_rate_dps = 0.0;
rotation_axis_vector = [1; 1; 0];
rotation_rate_rad = deg2rad(rotation_rate_dps);
rotation_axis = rotation_axis_vector / norm(rotation_axis_vector);

q_I2L_prev = [1; 0; 0; 0];
q_I2A_prev = [1; 0; 0; 0];
q_I2B_prev = [1; 0; 0; 0];
q_L2A_prev = [1; 0; 0; 0];
q_L2B_prev = [1; 0; 0; 0];
q_B2A_prev = [1; 0; 0; 0];
q_A2I_prev = [1; 0; 0; 0];
q_B2I_prev = [1; 0; 0; 0];

sensor_data = struct();
IMU_measurements = struct();
ST_measurements = struct();
GPS_measurements = struct();

camera_count = 0;

for k = 1:length(t1)
    current_time = t1(k);
    
    % 쿼터니언 계산
    r_hat = r_A_I(:,k)/norm(r_A_I(:,k));
    h_vec = cross(r_A_I(:,k),v_A_I(:,k));
    h_hat = h_vec/norm(h_vec);
    t_hat = cross(h_hat, r_hat);
    DCM_L2I = [r_hat, t_hat, h_hat];
    DCM_I2L = DCM_L2I';
    q_I2L = DCM2Quat(DCM_I2L);
    
    if norm(rotation_axis_vector) > 1e-6
        theta = rotation_rate_rad * current_time;
        q_L2A = [cos(theta/2); sin(theta/2) * rotation_axis];
        q_L2A = q_L2A / norm(q_L2A);
    else
        q_L2A = [1; 0; 0; 0];
    end
    
    q_L2B = CalculateAttitudeQuat_NoRoll(LVLH_pos(:,k), [0;0;0], orbit_normal_vec);
    
    if k > 1
        q_I2L = EnsQuatCont(q_I2L, q_I2L_prev);
        q_L2A = EnsQuatCont(q_L2A, q_L2A_prev);
        q_L2B = EnsQuatCont(q_L2B, q_L2B_prev);
    end
    q_I2L = q_I2L / norm(q_I2L);
    q_L2A = q_L2A / norm(q_L2A);
    q_L2B = q_L2B / norm(q_L2B);
    
    q_I2A = q2q_mult( q_L2A, q_I2L );
    q_I2B = q2q_mult( q_L2B, q_I2L );
    
    if k > 1
        q_I2A = EnsQuatCont(q_I2A, q_I2A_prev);
        q_I2B = EnsQuatCont(q_I2B, q_I2B_prev);
    end    
    
    q_I2A = q_I2A / norm(q_I2A);
    q_I2B = q_I2B / norm(q_I2B);
    R_I2A = GetDCM_QUAT(q_I2A);
    R_I2B = GetDCM_QUAT(q_I2B);
    
    q_A2I = inv_q(q_I2A);
    q_B2I = inv_q(q_I2B);
    
    omega_sIeL_observI = h_vec / (r_A_I(:,k)' * r_A_I(:,k));
    omega_sIeL_observL = DCM_I2L * omega_sIeL_observI;
    omega_sAeL_observL = rotation_axis * rotation_rate_rad;
    R_L2A = GetDCM_QUAT(q_L2A);
    R_A2L = R_L2A';
    R_L2B = GetDCM_QUAT(q_L2B);
    R_B2L = R_L2B';
    omega_sLeA_observA = -R_L2A * omega_sAeL_observL;
    
    omega_sLeI_observA = R_L2A * omega_sIeL_observL;
    omega_sIeA_observA = omega_sLeA_observA + omega_sLeI_observA;
    
    r_B_L = LVLH_pos(:,k);
    v_B_L = LVLH_vel(:,k);
    
    r_cross_v = cross(r_B_L, v_B_L);
    r_squared = r_B_L' * r_B_L;
    omega_r_L = r_cross_v / r_squared;
    
    omega_sBeL_observL = omega_r_L;
    
    R_L2B = GetDCM_QUAT(q_L2B);
    omega_sBeL_observB = R_L2B * omega_sBeL_observL;
    
    omega_sLeI_observB = R_L2B * omega_sIeL_observL;
    omega_sIeB_observB = omega_sBeL_observB + omega_sLeI_observB;
    
    omega_sLeB_observL = R_B2L * omega_sIeB_observB - omega_sIeL_observL;
    
    q_B2A = q2q_mult(q_L2A, inv_q(q_L2B));
    q_B2A = q_B2A / norm(q_B2A);
    if k > 1
        q_B2A = EnsQuatCont(q_B2A, q_B2A_prev);
    end
    
    R_B2A = GetDCM_QUAT(q_B2A);
    R_A2B = R_B2A';
    omega_sIeA_observB = R_A2B * omega_sIeA_observA;
    omega_sBeA_observB = omega_sIeA_observB - omega_sIeB_observB;
    omega_sAeB_observB = omega_sIeB_observB - omega_sIeA_observB;
    
    Xk_deputy = [r_B_I(:,k); v_B_I(:,k); q_I2B; omega_sIeB_observB];
    Xk_chief  = [r_A_I(:,k); v_A_I(:,k); q_I2A; omega_sIeA_observA];
    
    [z_imu_meas, z_imu_true, imu_biases, param] = measure_sensor_IMU( ...
        Xk_deputy, Xk_chief, Rk_IMU, param, dt);
    
    [z_st_meas, z_st_true, ST_params] = measure_sensor_ST_simple( ...
        Xk_deputy, [], [], ST_params, dt);
    
    sensor_data(k).time = current_time;
    sensor_data(k).LVLH_pos = LVLH_pos(:,k);
    sensor_data(k).q_L2A = q_L2A;
    sensor_data(k).q_L2B = q_L2B;
    sensor_data(k).q_I2B = q_I2B;
    sensor_data(k).q_I2A = q_I2A;
    sensor_data(k).q_B2I = q_B2I;
    sensor_data(k).q_A2I = q_A2I;
    sensor_data(k).q_B2A = q_B2A;
    sensor_data(k).omega_sLeB_observL = omega_sLeB_observL;
    sensor_data(k).omega_sLeA_observA = omega_sLeA_observA;
    sensor_data(k).omega_sBeA_observB = omega_sBeA_observB;
    sensor_data(k).omega_sAeB_observB = omega_sAeB_observB;
    sensor_data(k).omega_sIeB_observB = omega_sIeB_observB;
    sensor_data(k).omega_sIeA_observA = omega_sIeA_observA;
    
    IMU_measurements(k).time = current_time;
    IMU_measurements(k).acc_measured = z_imu_meas(1:3);
    IMU_measurements(k).gyro_measured = z_imu_meas(4:6);
    IMU_measurements(k).acc_true = z_imu_true(1:3);
    IMU_measurements(k).gyro_true = z_imu_true(4:6);
    IMU_measurements(k).biases = imu_biases;
    
    ST_measurements(k).time = current_time;
    ST_measurements(k).q_measured = z_st_meas;
    ST_measurements(k).q_true = z_st_true;
    
    [min_diff_gps, closest_gps_idx] = min(abs(gps_times_planned - current_time));
    
    if min_diff_gps < dt/2 && ~ismember(closest_gps_idx, processed_gps_idx)
        processed_gps_idx = [processed_gps_idx, closest_gps_idx];
        
        [z_gps_meas, z_gps_true, GPS_params] = measure_sensor_GPS_simple( ...
            Xk_deputy, [], [], GPS_params, dt);
        
        GPS_measurements(k).time = current_time;
        GPS_measurements(k).pos_measured = z_gps_meas(1:3);
        GPS_measurements(k).vel_measured = z_gps_meas(4:6);
        GPS_measurements(k).pos_true = z_gps_true(1:3);
        GPS_measurements(k).vel_true = z_gps_true(4:6);
        
        gps_count = gps_count + 1;
        
        if mod(gps_count, 500) == 0
            fprintf('  GPS 측정: %d회 (t=%.1fs, k=%d)\n', ...
                gps_count, current_time, k);
        end
    else
        GPS_measurements(k).time = current_time;
        GPS_measurements(k).pos_measured = [];
        GPS_measurements(k).vel_measured = [];
        GPS_measurements(k).pos_true = [];
        GPS_measurements(k).vel_true = [];
    end
    
    q_I2L_prev = q_I2L;
    q_I2A_prev = q_I2A;
    q_I2B_prev = q_I2B;
    q_L2A_prev = q_L2A;
    q_L2B_prev = q_L2B;
    q_B2A_prev = q_B2A;
    q_A2I_prev = q_A2I;
    q_B2I_prev = q_B2I;
    
    if exist('initial_pointcloud', 'var') && ~isempty(initial_pointcloud)
        [min_diff, closest_cam_idx] = min(abs(camera_times_planned - current_time));
        
        if min_diff < dt/2 && ~ismember(closest_cam_idx, processed_camera_idx)
            processed_camera_idx = [processed_camera_idx, closest_cam_idx];
            
            [z_meas, z_true, camera_params] = measure_sensor_camera_v2_2( ...
                Xk_deputy, Xk_chief, initial_pointcloud, camera_params, orbit_normal_vec, dt);
            
            z_camera_meas = z_meas;
            z_camera_true = z_true;
            
            sensor_data(k).z_measured = z_camera_meas;
            sensor_data(k).z_true = z_camera_true;
            
            K_cam = camera_params.intrinsic.K_ideal;
            
            rotation_angle = omega_earth_dps * current_time;
            shift_px = round((rotation_angle / 360) * size(earth_texture, 2));
            rotated_texture = circshift(earth_texture, [0, shift_px]);
            
            earth_image = render_earth_camera_view(...
                r_B_I(:,k), ...
                q_I2B, ...
                K_cam, ...
                camera_params, ...
                R_e, ...
                rotated_texture);
            
            sensor_data(k).earth_image = earth_image;
            
            if ~isempty(camera_params)
                sensor_data(k).camera_bias = [
                    camera_params.calib.fx_error;
                    camera_params.calib.fy_error;
                    camera_params.calib.cx_error;
                    camera_params.calib.cy_error;
                    camera_params.distortion.k1;
                    camera_params.distortion.k2;
                    camera_params.distortion.p1;
                    camera_params.distortion.p2
                ];
            else
                sensor_data(k).camera_bias = [];
            end
            
            camera_count = camera_count + 1;
            
            if mod(camera_count, 10) == 0
                fprintf('  카메라 측정: %d회 (t=%.1fs, k=%d)\n', ...
                    camera_count, current_time, k);
            end
        else
            sensor_data(k).z_measured = [];
            sensor_data(k).z_true = [];
            sensor_data(k).camera_bias = [];
            sensor_data(k).earth_image = [];
        end
    end
end
fprintf('센서 측정 완료.\n');
fprintf('  - IMU/ST: %d 측정\n', length(t1));
fprintf('  - GPS: %d 측정\n', gps_count);
fprintf('  - Camera: %d 측정\n', camera_count);

%% 색상 정의
colors = struct('radial', [0.8500, 0.3250, 0.0980], ...
                'intrack', [0, 0.4470, 0.7410], ...
                'crosstrack', [0.4660, 0.6740, 0.1880], ...
                'chief', [0.3010, 0.7450, 0.9330], ...
                'deputy', [0.6350, 0.0780, 0.1840], ...
                'gco', [0.8500, 0.3250, 0.0980], ...
                'sat_body_x', [1, 0, 0], ...
                'sat_body_y', [0, 1, 0], ...
                'sat_body_z', [0, 0, 1], ...
                'points', [0, 0.7, 1], ...
                'true_proj', [0, 0.8, 0.2], ...
                'meas_proj', [1, 0.2, 0.2]);

time_hours = t1 / 3600;
time_min = t1 / 60;

%% 제어 성능 시각화
figure('Name', 'Formation Control Performance', 'Position', [100, 100, 1400, 900]);

% LVLH 궤적 3D
subplot(2,3,1);
plot3(LVLH_pos(1,:), LVLH_pos(2,:), LVLH_pos(3,:), 'b-', 'LineWidth', 1.5);
hold on;
plot3(0, 0, 0, 'k+', 'MarkerSize', 15, 'LineWidth', 2);
plot3(0, -0.5, 0, 'ro', 'MarkerSize', 10, 'LineWidth', 2);
idx_trans = find(t1 >= t_transition, 1);
plot3(LVLH_pos(1,idx_trans), LVLH_pos(2,idx_trans), LVLH_pos(3,idx_trans), ...
      'gs', 'MarkerSize', 12, 'MarkerFaceColor', 'g');
grid on; axis equal;
xlabel('Radial [km]'); ylabel('In-track [km]'); zlabel('Cross-track [km]');
title('Formation Trajectory (LVLH)');
legend('Trajectory', 'Chief', 'Target', 'Transition', 'Location', 'best');
view(45, 25);

% LVLH 위치 성분
subplot(2,3,2);
plot(time_min, LVLH_pos', 'LineWidth', 1.5);
hold on;
xline(t_transition/60, 'k--', 'LineWidth', 2);
grid on;
xlabel('Time [min]'); ylabel('Position [km]');
title('LVLH Position Components');
legend('x (Radial)', 'y (In-track)', 'z (Cross-track)', 'Transition', 'Location', 'best');

% 상대 거리
subplot(2,3,3);
rel_dist = vecnorm(LVLH_pos, 2, 1);
target_dist = vecnorm(LVLH_pos - [0; -0.5; 0], 2, 1);
plot(time_min, rel_dist, 'b-', 'LineWidth', 1.5);
hold on;
plot(time_min, target_dist, 'r-', 'LineWidth', 1.5);
xline(t_transition/60, 'k--', 'LineWidth', 2);
grid on;
xlabel('Time [min]'); ylabel('Distance [km]');
title('Distance from Chief/Target');
legend('Distance from Chief', 'Distance from Target', 'Transition', 'Location', 'best');

% 제어 입력 (LVLH)
subplot(2,3,4);
plot(time_min, control_history.u_L'*1000, 'LineWidth', 1.5);
hold on;
xline(t_transition/60, 'k--', 'LineWidth', 2);
grid on;
xlabel('Time [min]'); ylabel('Control [m/s^2]');
title('Control Input (LVLH Frame)');
legend('u_x', 'u_y', 'u_z', 'Transition', 'Location', 'best');

% 제어 크기
subplot(2,3,5);
plot(time_min, control_history.thrust_mag*1000, 'LineWidth', 1.5);
hold on;
xline(t_transition/60, 'k--', 'LineWidth', 2);
grid on;
xlabel('Time [min]'); ylabel('Thrust Magnitude [m/s^2]');
title('Control Effort');

% 목표 추적 오차 (전환 후)
subplot(2,3,6);
idx_after = t1 >= t_transition;
error_pos = LVLH_pos(:, idx_after) - [0; -0.5; 0];
error_vel = LVLH_vel(:, idx_after);
error_mag = vecnorm(error_pos, 2, 1);
plot(time_min(idx_after), error_mag*1000, 'r-', 'LineWidth', 1.5);
grid on;
xlabel('Time [min]'); ylabel('Position Error [m]');
title('Tracking Error (After Transition)');

sgtitle('LQR Formation Control Performance', 'FontSize', 14, 'FontWeight', 'bold');

%% In-track vs GCO 비교 플롯
figure('Name', 'Formation Comparison', 'Position', [100, 100, 1200, 800]);

% XY 평면
subplot(2,2,1);
plot(LVLH_pos(1,:), LVLH_pos(2,:), 'b-', 'LineWidth', 1.5);
hold on;
plot(0, 0, 'k+', 'MarkerSize', 12, 'LineWidth', 2);
plot(0, -0.5, 'ro', 'MarkerSize', 10);
plot(LVLH_pos(1,idx_trans), LVLH_pos(2,idx_trans), 'gs', 'MarkerSize', 12);
grid on; axis equal;
xlabel('Radial [km]'); ylabel('In-track [km]');
title('Radial vs In-track');
legend('Trajectory', 'Chief', 'Target', 'Transition');

% XZ 평면
subplot(2,2,2);
plot(LVLH_pos(1,:), LVLH_pos(3,:), 'b-', 'LineWidth', 1.5);
hold on;
plot(0, 0, 'k+', 'MarkerSize', 12, 'LineWidth', 2);
plot(LVLH_pos(1,idx_trans), LVLH_pos(3,idx_trans), 'gs', 'MarkerSize', 12);
grid on; axis equal;
xlabel('Radial [km]'); ylabel('Cross-track [km]');
title('Radial vs Cross-track');

% YZ 평면
subplot(2,2,3);
plot(LVLH_pos(2,:), LVLH_pos(3,:), 'b-', 'LineWidth', 1.5);
hold on;
plot(0, 0, 'k+', 'MarkerSize', 12, 'LineWidth', 2);
plot(-0.5, 0, 'ro', 'MarkerSize', 10);
plot(LVLH_pos(2,idx_trans), LVLH_pos(3,idx_trans), 'gs', 'MarkerSize', 12);
grid on; axis equal;
xlabel('In-track [km]'); ylabel('Cross-track [km]');
title('In-track vs Cross-track');

% 3D
subplot(2,2,4);
plot3(LVLH_pos(1,:), LVLH_pos(2,:), LVLH_pos(3,:), 'b-', 'LineWidth', 1.5);
hold on;
plot3(0, 0, 0, 'k+', 'MarkerSize', 12, 'LineWidth', 2);
plot3(0, -0.5, 0, 'ro', 'MarkerSize', 10);
plot3(LVLH_pos(1,idx_trans), LVLH_pos(2,idx_trans), LVLH_pos(3,idx_trans), ...
      'gs', 'MarkerSize', 12);
grid on; axis equal;
xlabel('x [km]'); ylabel('y [km]'); zlabel('z [km]');
title('3D View');
view(45, 25);

sgtitle('GCO to In-track Formation Transition', 'FontSize', 14, 'FontWeight', 'bold');

%% 데이터 저장
fprintf('\n=== 데이터 저장을 시작합니다 ===\n');

EXPORT_DIR = fullfile(pwd, 'sim_data');
if ~exist(EXPORT_DIR, 'dir'); mkdir(EXPORT_DIR); end
STAMP = datestr(now, 'yyyymmdd_HHMMSS');
SAVEPATH = fullfile(EXPORT_DIR, sprintf('sim_data_lqr_%s.mat', STAMP));

USE_SINGLE = false;

time = struct();
time.t_s = t1(:)';
time.t_min = (t1/60).';
time.t_hr = (t1/3600).';

traj = struct();
traj.r_A_I = r_A_I; traj.v_A_I = v_A_I;
traj.r_B_I = r_B_I; traj.v_B_I = v_B_I;
traj.r_B_L = LVLH_pos;
traj.v_B_L = LVLH_vel;
traj.orbit_normal_vec = orbit_normal_vec(:);

quat = struct();
quat.q_I2B = [sensor_data.q_I2B];
quat.q_L2A = [sensor_data.q_L2A];
quat.q_L2B = [sensor_data.q_L2B];
quat.q_I2A = [sensor_data.q_I2A];
quat.q_B2A = [sensor_data.q_B2A];
quat.omega_sIeB_observB = [sensor_data.omega_sIeB_observB];
quat.omega_sLeB_observL = [sensor_data.omega_sLeB_observL];
quat.omega_sLeA_observA = [sensor_data.omega_sLeA_observA];
quat.omega_sBeA_observB = [sensor_data.omega_sBeA_observB];
quat.omega_sIeA_observA = [sensor_data.omega_sIeA_observA];

imu = struct();
imu.acc_meas = [IMU_measurements.acc_measured];
imu.gyro_meas = [IMU_measurements.gyro_measured];
imu.acc_true = [IMU_measurements.acc_true];
imu.gyro_true = [IMU_measurements.gyro_true];
imu.biases = [IMU_measurements.biases];

st = struct();
st.q_meas = [ST_measurements.q_measured];
st.q_true = [ST_measurements.q_true];
st_att_error_deg = zeros(1, length(t1));
for k = 1:length(t1)
    q_err = q2q_mult(ST_measurements(k).q_measured, inv_q(ST_measurements(k).q_true));
    if q_err(1) < 0, q_err = -q_err; end
    angle_err = 2 * acos(min(1, abs(q_err(1))));
    st_att_error_deg(k) = rad2deg(angle_err);
end
st.err_deg = st_att_error_deg(:)';

gps = struct();
gps_valid_idx = arrayfun(@(x) ~isempty(x.pos_measured), GPS_measurements);
gps_times = [GPS_measurements(gps_valid_idx).time];
pos_gps_meas = [GPS_measurements(gps_valid_idx).pos_measured];
pos_gps_true = [GPS_measurements(gps_valid_idx).pos_true];
vel_gps_meas = [GPS_measurements(gps_valid_idx).vel_measured];
vel_gps_true = [GPS_measurements(gps_valid_idx).vel_true];
pos_error = pos_gps_meas - pos_gps_true;
vel_error = vel_gps_meas - vel_gps_true;
gps.times_s = gps_times(:)';
gps.pos_meas = pos_gps_meas;
gps.vel_meas = vel_gps_meas;
gps.pos_true = pos_gps_true;
gps.vel_true = vel_gps_true;
gps.pos_err_mag = (vecnorm(pos_error, 2, 1) * 1000)';
gps.vel_err_mag = (vecnorm(vel_error, 2, 1) * 1000)';

cam_has = arrayfun(@(s) ~isempty(s.z_measured), sensor_data);
cam_idx = find(cam_has);
cam_meas = cell(1, numel(cam_idx));
cam_true = cell(1, numel(cam_idx));
cam_earth_images = cell(1, numel(cam_idx));

for ii = 1:numel(cam_idx)
    k = cam_idx(ii);
    cam_meas{ii} = sensor_data(k).z_measured;
    cam_true{ii} = sensor_data(k).z_true;
    if isfield(sensor_data(k), 'earth_image') && ~isempty(sensor_data(k).earth_image)
        cam_earth_images{ii} = sensor_data(k).earth_image;
    else
        cam_earth_images{ii} = [];
    end
end

camera = struct();
if exist('initial_pointcloud','var') && ~isempty(initial_pointcloud)
    camera.pointcloud = initial_pointcloud;
else
    camera.pointcloud = [];
end
if exist('camera_params','var') && ~isempty(camera_params)
    camera.params = camera_params;
else
    camera.params = struct();
end
camera.times_s = t1(cam_idx).';
camera.z_measured = cam_meas;
camera.z_true = cam_true;
camera.earth_images = cam_earth_images;
camera.biases = [];
for k = 1:length(sensor_data)
    if isfield(sensor_data(k), 'camera_bias') && ~isempty(sensor_data(k).camera_bias)
        if isempty(camera.biases)
            camera.biases = sensor_data(k).camera_bias;
        else
            camera.biases = [camera.biases, sensor_data(k).camera_bias];
        end
    end
end

% 제어 데이터 추가
control = struct();
control.u_I = control_history.u_I;
control.u_L = control_history.u_L;
control.thrust_mag = control_history.thrust_mag;
control.K = K;
control.Q = Q;
control.R = R;
control.t_transition = t_transition;

const = struct('Mu',Mu,'R_e',R_e,'J2',J2,'param',param, ...
               'Rk_IMU',Rk_IMU,'Rk_ST',Rk_ST);
meta = struct('dt',dt,'rate_imu_hz',1/dtdtdt,'rate_gps_hz',1,'rate_camera_s',camera_interval, ...
              'convention','scalar-first, active rotation', ...
              'frames','I/L/A/B','generated_at',STAMP, ...
              'control_type','LQR', 't_transition_s', t_transition);

if USE_SINGLE
    toSingle = @(M) single(M);
    traj.r_A_I = toSingle(traj.r_A_I); traj.v_A_I = toSingle(traj.v_A_I);
    traj.r_B_I = toSingle(traj.r_B_I); traj.v_B_I = toSingle(traj.v_B_I);
    traj.r_B_L = toSingle(traj.r_B_L); traj.v_B_L = toSingle(traj.v_B_L);
    control.u_I = toSingle(control.u_I);
    control.u_L = toSingle(control.u_L);
    control.thrust_mag = toSingle(control.thrust_mag);
end

save(SAVEPATH,'meta','const','time','traj','quat','imu','st','gps','camera','control','-v7.3');
info = dir(SAVEPATH);
fprintf('저장 완료: %s (%.2f MB)\n', SAVEPATH, info.bytes/1024/1024);

fprintf('\n=== LQR 제어 시뮬레이션 완료 ===\n');
fprintf('GCO 구간: %.1f초 (%.2f 궤도)\n', t_transition, t_transition/T_Op);
fprintf('In-track 구간: %.1f초 (%.2f 궤도)\n', tt-t_transition, (tt-t_transition)/T_Op);
fprintf('최종 위치 오차: %.2f m\n', norm(LVLH_pos(:,end) - [0; -0.5; 0])*1000);
fprintf('최종 속도 오차: %.2f mm/s\n', norm(LVLH_vel(:,end))*1e6);
% 
% %% === 별 데이터 생성 (TRUE + NOISE) ===
% fprintf('\n=== 별 데이터 생성 시작 ===\n');
% 
% % 출력 디렉토리
% star_data_base = 'D:\star_tracker_test\main_pj_code\sim_data\star_data';
% star_data_true_dir = fullfile(star_data_base, 'true');
% star_data_noise_dir = fullfile(star_data_base, 'noise');
% if ~exist(star_data_true_dir, 'dir'), mkdir(star_data_true_dir); end
% if ~exist(star_data_noise_dir, 'dir'), mkdir(star_data_noise_dir); end
% 
% % 카탈로그 로드
% catalog_file = 'D:\star_tracker_test\main_pj_code\model\sensor\star_tracker\filtered_catalogue\Hipparcos_Below_6.0.csv';
% opts = detectImportOptions(catalog_file);
% opts.VariableNamingRule = 'preserve';
% star_catalogue = readtable(catalog_file, opts);
% fprintf('카탈로그 로드: %d개 별\n', height(star_catalogue));
% 
% % 센서 파라미터 (Star Tracker)
% st_sensor.myu = 2e-6;
% st_sensor.f = 0.01042;
% st_sensor.l = 1280;
% st_sensor.w = 720;
% 
% FOVy = rad2deg(2 * atan((st_sensor.myu*st_sensor.w/2) / st_sensor.f));
% FOVx = rad2deg(2 * atan((st_sensor.myu*st_sensor.l/2) / st_sensor.f));
% fprintf('Star Tracker FOV: %.1f x %.1f degrees\n', FOVx, FOVy);
% 
% % 노이즈 레벨
% sigma_pixel = 0.3;
% fprintf('픽셀 노이즈: %.2f pixels\n\n', sigma_pixel);
% 
% % R_B2ST (Body → Star Tracker frame)
% R_B2ST = [1, 0, 0; 0, 0, 1; 0, -1, 0];
% 
% % 별 카탈로그 데이터
% ra_stars = star_catalogue.RA;
% de_stars = star_catalogue.DE;
% magnitudes = star_catalogue.Magnitude;
% star_ids = star_catalogue.('Star ID');
% 
% pixel_per_length = 1 / st_sensor.myu;
% R_fov = sqrt(deg2rad(FOVx)^2 + deg2rad(FOVy)^2) / 2;
% 
% % 결과 저장
% star_data_true_all = struct();
% star_data_noise_all = struct();
% 
% fprintf('별 데이터 생성 중...\n');
% for k = 1:length(t1)
%     % Deputy 위치 (ECI)
%     r_B_I_k = r_B_I(:, k);
% 
%     % Deputy 자세 (ECI → Body)
%     q_I2B_k = quat.q_I2B(:, k);
%     R_I2B_k = GetDCM_QUAT(q_I2B_k);
%     R_I2ST = R_B2ST * R_I2B_k;
%     M = R_I2ST';
%     M_transpose = M';
% 
%     % ST 광축 방향 (ECI)
%     z_st = M(:, 3);
%     ra = atan2(z_st(2), z_st(1));
%     dec = asin(z_st(3));
%     if ra < 0, ra = ra + 2*pi; end
% 
%     % FOV 검색 영역
%     alpha_start = ra - R_fov/cos(dec);
%     alpha_end = ra + R_fov/cos(dec);
%     delta_start = dec - R_fov;
%     delta_end = dec + R_fov;
% 
%     % RA Wraparound
%     if alpha_start < 0 || alpha_end > 2*pi
%         if alpha_start < 0
%             star_within_ra = ((alpha_start + 2*pi) <= ra_stars) | (ra_stars <= alpha_end);
%         else
%             star_within_ra = (alpha_start <= ra_stars) | (ra_stars <= (alpha_end - 2*pi));
%         end
%     else
%         star_within_ra = (alpha_start <= ra_stars) & (ra_stars <= alpha_end);
%     end
% 
%     star_within_de = (delta_start <= de_stars) & (de_stars <= delta_end);
%     stars_in_fov = star_within_ra & star_within_de;
% 
%     ra_i = ra_stars(stars_in_fov);
%     de_i = de_stars(stars_in_fov);
%     mag_i = magnitudes(stars_in_fov);
%     id_i = star_ids(stars_in_fov);
% 
%     % 지구 가림 체크
%     earth_occluded = false(length(ra_i), 1);
%     for i = 1:length(ra_i)
%         star_dir = [cos(ra_i(i))*cos(de_i(i));
%                     sin(ra_i(i))*cos(de_i(i));
%                     sin(de_i(i))];
% 
%         a = 1;
%         b = 2 * dot(r_B_I_k, star_dir);
%         c = dot(r_B_I_k, r_B_I_k) - R_e^2;
%         discriminant = b^2 - 4*a*c;
% 
%         if discriminant >= 0
%             t1_int = (-b - sqrt(discriminant)) / (2*a);
%             t2_int = (-b + sqrt(discriminant)) / (2*a);
%             if t1_int > 0 || t2_int > 0
%                 earth_occluded(i) = true;
%             end
%         end
%     end
% 
%     ra_i = ra_i(~earth_occluded);
%     de_i = de_i(~earth_occluded);
%     mag_i = mag_i(~earth_occluded);
%     id_i = id_i(~earth_occluded);
% 
%     % === TRUE 버전: 노이즈 없음 ===
%     pixel_coords_true = [];
%     filtered_mag_true = [];
%     filtered_ids_true = [];
% 
%     for i = 1:length(ra_i)
%         dir_vector = [cos(ra_i(i))*cos(de_i(i));
%                       sin(ra_i(i))*cos(de_i(i));
%                       sin(de_i(i))];
%         star_sensor = M_transpose * dir_vector;
% 
%         x = st_sensor.f * (star_sensor(1) / star_sensor(3));
%         y = st_sensor.f * (star_sensor(2) / star_sensor(3));
% 
%         x_pixel = x / st_sensor.myu;
%         y_pixel = y / st_sensor.myu;
% 
%         if abs(x_pixel) <= st_sensor.l/2 && abs(y_pixel) <= st_sensor.w/2
%             pixel_coords_true = [pixel_coords_true; x_pixel, y_pixel];
%             filtered_mag_true = [filtered_mag_true; mag_i(i)];
%             filtered_ids_true = [filtered_ids_true; id_i(i)];
%         end
%     end
% 
%     star_data_true_all(k).frame = k;
%     star_data_true_all(k).time = t1(k);
%     star_data_true_all(k).ra = ra;
%     star_data_true_all(k).dec = dec;
%     star_data_true_all(k).num_stars = length(filtered_mag_true);
%     star_data_true_all(k).star_ids = filtered_ids_true;
%     star_data_true_all(k).magnitudes = filtered_mag_true;
%     star_data_true_all(k).pixel_coords = pixel_coords_true;
% 
%     % === NOISE 버전: 픽셀 노이즈 추가 ===
%     pixel_coords_noise = [];
%     filtered_mag_noise = [];
%     filtered_ids_noise = [];
% 
%     for i = 1:length(ra_i)
%         dir_vector = [cos(ra_i(i))*cos(de_i(i));
%                       sin(ra_i(i))*cos(de_i(i));
%                       sin(de_i(i))];
%         star_sensor = M_transpose * dir_vector;
% 
%         x = st_sensor.f * (star_sensor(1) / star_sensor(3));
%         y = st_sensor.f * (star_sensor(2) / star_sensor(3));
% 
%         x_pixel = (x / st_sensor.myu) + sigma_pixel * randn();
%         y_pixel = (y / st_sensor.myu) + sigma_pixel * randn();
% 
%         if abs(x_pixel) <= st_sensor.l/2 && abs(y_pixel) <= st_sensor.w/2
%             pixel_coords_noise = [pixel_coords_noise; x_pixel, y_pixel];
%             filtered_mag_noise = [filtered_mag_noise; mag_i(i)];
%             filtered_ids_noise = [filtered_ids_noise; id_i(i)];
%         end
%     end
% 
%     star_data_noise_all(k).frame = k;
%     star_data_noise_all(k).time = t1(k);
%     star_data_noise_all(k).ra = ra;
%     star_data_noise_all(k).dec = dec;
%     star_data_noise_all(k).num_stars = length(filtered_mag_noise);
%     star_data_noise_all(k).star_ids = filtered_ids_noise;
%     star_data_noise_all(k).magnitudes = filtered_mag_noise;
%     star_data_noise_all(k).pixel_coords = pixel_coords_noise;
% 
%     if mod(k, 1000) == 0
%         fprintf('  진행: %d/%d (%.1f%%)\n', k, length(t1), k/length(t1)*100);
%     end
% end
% 
% fprintf('별 데이터 생성 완료!\n');
% 
% % 통계
% num_stars_true = [star_data_true_all.num_stars];
% num_stars_noise = [star_data_noise_all.num_stars];
% 
% fprintf('\n=== TRUE 데이터 통계 ===\n');
% fprintf('평균: %.1f개\n', mean(num_stars_true));
% fprintf('최소/최대: %d / %d개\n', min(num_stars_true), max(num_stars_true));
% 
% fprintf('\n=== NOISE 데이터 통계 (σ=%.2f pixels) ===\n', sigma_pixel);
% fprintf('평균: %.1f개\n', mean(num_stars_noise));
% fprintf('최소/최대: %d / %d개\n', min(num_stars_noise), max(num_stars_noise));
% 
% % 저장
% save_file_true = fullfile(star_data_true_dir, sprintf('star_data_true_%s.mat', STAMP));
% save_file_noise = fullfile(star_data_noise_dir, sprintf('star_data_noise%.1f_%s.mat', sigma_pixel, STAMP));
% 
% save(save_file_true, 'star_data_true_all', 'st_sensor', '-v7.3');
% save(save_file_noise, 'star_data_noise_all', 'sigma_pixel', 'st_sensor', '-v7.3');
% 
% fprintf('\n저장 완료:\n');
% fprintf('  TRUE:  %s\n', save_file_true);
% fprintf('  NOISE: %s\n', save_file_noise);
% 
% %% === Star Catalog 빌드 및 저장 ===
% fprintf('\n=== Star Catalog 빌드 ===\n');
% 
% catalog_fast_file = fullfile(pwd, 'star_catalog_fast.mat');
% 
% if ~exist(catalog_fast_file, 'file')
%     fprintf('카탈로그 파일 없음. 새로 빌드합니다...\n');
% 
%     % k-vector 카탈로그 구축
%     FOV_deg = 14;
%     mag_threshold = 6.0;
%     catalog_data = build_star_catalog_kvector(catalog_file, FOV_deg, mag_threshold);
% 
%     % 배열로 변환
%     n_pairs = length(catalog_data.sorted_pairs);
%     pairs_I = zeros(n_pairs, 1, 'uint16');
%     pairs_J = zeros(n_pairs, 1, 'uint16');
%     pairs_angle = zeros(n_pairs, 1, 'single');
% 
%     for i = 1:n_pairs
%         pairs_I(i) = catalog_data.sorted_pairs(i).I;
%         pairs_J(i) = catalog_data.sorted_pairs(i).J;
%         pairs_angle(i) = catalog_data.sorted_pairs(i).angle;
%     end
% 
%     % 최적화 구조체
%     catalog_fast = struct();
%     catalog_fast.N_stars = catalog_data.N_stars;
%     catalog_fast.N_pairs = catalog_data.N_pairs;
%     catalog_fast.FOV_rad = catalog_data.FOV_rad;
%     catalog_fast.star_RA = single(catalog_data.star_catalog.RA);
%     catalog_fast.star_DEC = single(catalog_data.star_catalog.DEC);
%     catalog_fast.star_Mag = single(catalog_data.star_catalog.Magnitude);
%     catalog_fast.star_ID = uint32(catalog_data.star_catalog.ID);
%     catalog_fast.r_I = single(catalog_data.r_I);
%     catalog_fast.pairs_I = pairs_I;
%     catalog_fast.pairs_J = pairs_J;
%     catalog_fast.pairs_angle = pairs_angle;
%     catalog_fast.k_vector = uint32(catalog_data.k_vector);
%     catalog_fast.m = catalog_data.m;
%     catalog_fast.q = catalog_data.q;
% 
%     save(catalog_fast_file, 'catalog_fast', '-v7.3');
%     file_info = dir(catalog_fast_file);
%     fprintf('카탈로그 저장: %s (%.2f MB)\n', catalog_fast_file, file_info.bytes/1e6);
% else
%     fprintf('기존 카탈로그 존재: %s\n', catalog_fast_file);
% end
% 
% fprintf('\n=== 전체 데이터 생성 완료 ===\n');