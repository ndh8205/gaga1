clc; clear all; close all;
addpath(genpath('D:\Gaimsat_pj\matsim'));

%% 데이터 로드
data_dir = fullfile(pwd, 'sim_data');
files = dir(fullfile(data_dir, 'sim_data_lqr_*.mat'));  % ← 파일명 수정
if isempty(files)
    error('데이터 파일을 찾을 수 없습니다: %s', fullfile(data_dir, 'sim_data_ver2_*.mat'));
end
load(fullfile(data_dir, files(end).name));
fprintf('로드: %s\n', files(end).name);

%% 로드된 데이터에서 필요한 변수 복원
% 시간
t1 = time.t_s';
time_min = time.t_min';

% 궤도
LVLH_pos = traj.r_B_L;
LVLH_vel = traj.v_B_L;
orbit_normal_vec = traj.orbit_normal_vec;
r_A_I = traj.r_A_I;
v_A_I = traj.v_A_I;
r_B_I = traj.r_B_I;
v_B_I = traj.v_B_I;

% 파라미터
Mu = const.Mu;
R_e = const.R_e;
J2 = const.J2;
param = const.param;
dt = meta.dt;
r_init_B = 1.25; % GCO 반경

% Point cloud
initial_pointcloud = camera.pointcloud;
num_points = size(initial_pointcloud, 2);

% 카메라 파라미터
camera_params = camera.params;

%% sensor_data 재구성
sensor_data = struct();
N = length(t1);
cam_idx_all = find(arrayfun(@(t) any(abs(camera.times_s - t) < dt/2), t1));
cam_count = 1;
for k = 1:N
    sensor_data(k).time = t1(k);
    sensor_data(k).LVLH_pos = LVLH_pos(:,k);
    sensor_data(k).q_L2A = quat.q_L2A(:,k);
    sensor_data(k).q_L2B = quat.q_L2B(:,k);
    sensor_data(k).q_I2B = quat.q_I2B(:,k);
    sensor_data(k).q_I2A = quat.q_I2A(:,k);
    sensor_data(k).q_B2A = quat.q_B2A(:,k);
    sensor_data(k).DCM_L2B = GetDCM_QUAT(quat.q_L2B(:,k));
    
    % 필드명 수정
    sensor_data(k).omega_B_L = quat.omega_sLeB_observL(:,k); % ω^L_{B/L}
    sensor_data(k).omega_B_I = quat.omega_sIeB_observB(:,k); % ω^B_{B/I}
    
    % 추가 각속도 필드
    sensor_data(k).omega_sLeA_observA = quat.omega_sLeA_observA(:,k); % ω^A_{A/L}
    sensor_data(k).omega_sBeA_observB = quat.omega_sBeA_observB(:,k); % ω^B_{A/B}
    
    % 카메라 측정값 매칭
    if ismember(k, cam_idx_all) && cam_count <= length(camera.z_measured)
        sensor_data(k).z_measured = camera.z_measured{cam_count};
        sensor_data(k).z_true = camera.z_true{cam_count};
        
        % ========== earth_image 추가 ==========
        if isfield(camera, 'earth_images') && cam_count <= length(camera.earth_images)
            sensor_data(k).earth_image = camera.earth_images{cam_count};
        else
            sensor_data(k).earth_image = [];
        end
        % =====================================
        
        cam_count = cam_count + 1;
    else
        sensor_data(k).z_measured = [];
        sensor_data(k).z_true = [];
        sensor_data(k).earth_image = [];  % ← 추가
    end
end
fprintf('sensor_data 재구성 완료: %d 프레임\n', N);

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
                'true_proj', [1, 1, 1], ...
                'meas_proj', [1, 1, 1]);

%% q_L2B 오일러 각 플롯
euler_angles = zeros(3, length(t1));
for k = 1:length(t1)
    q_L2B = sensor_data(k).q_L2B;
    euler = Quat2Euler(q_L2B); % [phi; theta; psi]
    euler_angles(:, k) = euler * 180/pi; % rad to deg
end

%% 13. 통합 센서 애니메이션

% ani_3d_lvlh_formation(sensor_data, initial_pointcloud, camera_params, ...
%     LVLH_pos, euler_angles, colors, r_init_B, t1);
% 
% ani_combined_lvlh_attitude2(sensor_data, initial_pointcloud, camera_params, ...
%     LVLH_pos, euler_angles, colors, r_init_B, t1, orbit_normal_vec);
% 
% ani_eci_simple(r_A_I, r_B_I, colors, R_e, t1, 5);

% ani_eci_simple2(r_A_I, r_B_I, colors, R_e, t1, 5);

% ani_eci_fixed_view(r_A_I, r_B_I, v_A_I, sensor_data, colors, R_e, t1, 60.0);
% 
% ani_eci_deputy_fixed(r_A_I, r_B_I, v_A_I, sensor_data, colors, R_e, t1, 10);
% 
% ani_eci_deputy_fixed_lvlh(r_A_I, r_B_I, v_A_I, sensor_data, colors, R_e, t1, 10);
% 
ani_3d_lvlh_formation_v2(sensor_data, initial_pointcloud, camera_params, ...
                         LVLH_pos, euler_angles, colors, r_init_B, t1)
