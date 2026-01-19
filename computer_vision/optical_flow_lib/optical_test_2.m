clc; clear all; close all;
addpath(genpath('C:\Users\USER\Desktop\relative2'));

%% 데이터 로드
data_dir = fullfile(pwd, 'sim_data');
files = dir(fullfile(data_dir, 'sim_data_ver2_*.mat'));
latest_file = fullfile(data_dir, files(end).name);
load(latest_file);

fprintf('=== 옵티컬 플로우 조건 진단 ===\n\n');

%% 카메라 측정 시점 분석
dt_cam = diff(camera.times_s);
fprintf('카메라 측정 간격: %.1f초\n', mean(dt_cam));

%% 각 프레임 간 변화량 분석
for i = 1:min(5, length(camera.times_s)-1)
    t1 = camera.times_s(i);
    t2 = camera.times_s(i+1);
    dt = t2 - t1;
    
    [~, idx1] = min(abs(time.t_s - t1));
    [~, idx2] = min(abs(time.t_s - t2));
    
    % 회전 변화
    omega_B = quat.omega_sIeB_observB(:, idx1);
    q1 = quat.q_I2B(:, idx1);
    q2 = quat.q_I2B(:, idx2);
    q_diff = q2q_mult(q2, inv_q(q1));
    angle_change = 2 * acos(min(1, abs(q_diff(1)))) * 180/pi;
    
    % 병진 변화 (LVLH)
    r1 = traj.r_B_L(:, idx1);
    r2 = traj.r_B_L(:, idx2);
    dr = r2 - r1;
    
    % 평균 거리
    avg_dist = (norm(r1) + norm(r2)) / 2;
    
    % 픽셀 움직임 예측
    z1 = camera.z_measured{i};
    z2 = camera.z_measured{i+1};
    if ~isempty(z1) && ~isempty(z2)
        n_common = min(size(z1,2), size(z2,2));
        pixel_motion = vecnorm(z2(1:2, 1:n_common) - z1(1:2, 1:n_common), 2, 1);
        avg_pixel_motion = mean(pixel_motion);
    else
        avg_pixel_motion = NaN;
    end
    
    fprintf('\n프레임 %d → %d (dt=%.1fs):\n', i, i+1, dt);
    fprintf('  자세 변화: %.2f도\n', angle_change);
    fprintf('  각속도: [%.4f, %.4f, %.4f] deg/s\n', omega_B*180/pi);
    fprintf('  병진 이동: [%.4f, %.4f, %.4f] km\n', dr);
    fprintf('  병진 크기: %.4f km (%.2f m/s)\n', norm(dr), norm(dr)*1000/dt);
    fprintf('  평균 거리: %.4f km\n', avg_dist);
    fprintf('  병진/거리 비율: %.2e\n', norm(dr)/avg_dist);
    fprintf('  평균 픽셀 이동: %.1f pixels\n', avg_pixel_motion);
end

%% 병진 vs 회전 기여도 분석
fprintf('\n=== 병진 vs 회전 기여도 ===\n');

% 첫 측정 시점
i = 1;
t1 = camera.times_s(i);
t2 = camera.times_s(i+1);
[~, idx1] = min(abs(time.t_s - t1));
[~, idx2] = min(abs(time.t_s - t2));
dt_cam = t2 - t1;

% Ground truth
omega_B = quat.omega_sIeB_observB(:, idx1);
r_B_L_1 = traj.r_B_L(:, idx1);
r_B_L_2 = traj.r_B_L(:, idx2);
v_B_L = (r_B_L_2 - r_B_L_1) / dt_cam;

fprintf('\n각속도 (B frame): [%.4f, %.4f, %.4f] deg/s\n', omega_B*180/pi);
fprintf('병진속도 (LVLH): [%.4f, %.4f, %.4f] km/s (%.2f m/s)\n', ...
    v_B_L, norm(v_B_L)*1000);

% 카메라 파라미터
K = camera.params.intrinsic.K_ideal;
f_pixels = K(1,1);

% 대표 특징점에서 픽셀 속도 기여도
z1 = camera.z_measured{i};
if ~isempty(z1) && size(z1, 2) > 0
    % 중심 근처 점 선택
    center_px = [512; 512];
    dists = vecnorm(z1(1:2, :) - center_px, 2, 1);
    [~, center_idx] = min(dists);
    
    uv_test = z1(1:2, center_idx);
    
    % 변환 체인 구성
    r_A_I = traj.r_A_I(:, idx1);
    v_A_I = traj.v_A_I(:, idx1);
    r_hat = r_A_I / norm(r_A_I);
    h_vec = cross(r_A_I, v_A_I);
    h_hat = h_vec / norm(h_vec);
    t_hat = cross(h_hat, r_hat);
    R_L2I = [r_hat, t_hat, h_hat];
    R_I2L = R_L2I';
    
    q_L2A = quat.q_L2A(:, idx1);
    q_I2B = quat.q_I2B(:, idx1);
    R_L2A = GetDCM_QUAT(q_L2A);
    R_A2L = R_L2A';
    R_I2B = GetDCM_QUAT(q_I2B);
    R_B2C = [1, 0, 0; 0, 0, -1; 0, 1, 0];
    R_L2B = R_I2B * R_L2I;
    R_L2C = R_B2C * R_L2B;
    
    % 특징점 3D 좌표
    points_A = camera.pointcloud;
    points_L = R_A2L * points_A(:, center_idx);
    point_C = R_L2C * (points_L - r_B_L_1);
    depth = point_C(3);
    
    % 정규화 좌표
    K_inv = inv(K);
    p_n = K_inv * [uv_test; 1];
    
    % 회전 기여: -ω × p_n
    omega_C = R_B2C * R_L2B * omega_B;
    rotation_contrib = -cross(omega_C, p_n);
    rotation_pixel_velocity = [f_pixels * rotation_contrib(1); 
                                f_pixels * rotation_contrib(2)];
    
    % 병진 기여: (1/d)[v - (v·p_n)p_n]
    v_C = R_L2C * v_B_L;
    v_dot_pn = v_C' * p_n;
    translation_contrib = (v_C - v_dot_pn * p_n) / depth;
    translation_pixel_velocity = [f_pixels * translation_contrib(1);
                                   f_pixels * translation_contrib(2)];
    
    fprintf('\n중심 근처 특징점 (픽셀 [%.1f, %.1f], 깊이 %.3f km):\n', ...
        uv_test, depth);
    fprintf('  회전 기여:   [%+.2f, %+.2f] px/s (크기: %.2f px/s)\n', ...
        rotation_pixel_velocity, norm(rotation_pixel_velocity));
    fprintf('  병진 기여:   [%+.2f, %+.2f] px/s (크기: %.2f px/s)\n', ...
        translation_pixel_velocity, norm(translation_pixel_velocity));
    fprintf('  전체 예상:   [%+.2f, %+.2f] px/s\n', ...
        rotation_pixel_velocity + translation_pixel_velocity);
    fprintf('  병진/회전 비율: %.2f%%\n', ...
        100 * norm(translation_pixel_velocity) / norm(rotation_pixel_velocity));
end

fprintf('\n=== 결론 ===\n');
fprintf('✗ 측정 간격 60초는 옵티컬 플로우에 너무 길다 (권장: <0.1초)\n');
fprintf('✗ 자세 변화 ~78도는 작은 움직임 가정을 위반한다 (권장: <1도)\n');
fprintf('✗ 병진 성분이 무시할 수 없는 수준이다\n');
fprintf('\n권장 사항:\n');
fprintf('1. 카메라 프레임 레이트를 높인다 (>10Hz)\n');
fprintf('2. 병진 성분을 포함한 6-DOF 추정 사용\n');
fprintf('3. 또는 PnP 방식의 자세 추정 후 미분 사용\n');