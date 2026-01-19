clc; clear all; close all;
addpath(genpath('C:\Users\USER\Desktop\relative2'));

%% 데이터 로드
data_dir = fullfile(pwd, 'sim_data');
files = dir(fullfile(data_dir, 'sim_data_ver2_*.mat'));
if isempty(files)
    error('시뮬레이션 데이터 파일을 찾을 수 없습니다.');
end
latest_file = fullfile(data_dir, files(end).name);
fprintf('데이터 로드 중: %s\n', files(end).name);
load(latest_file);

T_total = length(time.t_s);
dt = meta.dt;

fprintf('총 시간: %.1f초 (%.1f분)\n', time.t_s(end), time.t_s(end)/60);
fprintf('카메라 측정 횟수: %d\n', length(camera.times_s));

%% 카메라 파라미터
if ~isempty(camera.params) && isfield(camera.params, 'intrinsic')
    K = camera.params.intrinsic.K_ideal;
else
    img_width = 1024;
    fov = 3.6 * pi/180;
    f = img_width / (2 * tan(fov/2));
    K = [f, 0, img_width/2;
         0, f, img_width/2;
         0, 0, 1];
end

K_inv = inv(K);
R_B2C = [1, 0, 0; 0, 0, -1; 0, 1, 0];  % Body → Camera (검증됨)

fprintf('\n카메라 파라미터:\n');
fprintf('  fx = %.2f, fy = %.2f\n', K(1,1), K(2,2));
fprintf('  cx = %.2f, cy = %.2f\n', K(1,3), K(2,3));

%% Point Cloud (Chief body frame A)
if isempty(camera.pointcloud)
    error('Point cloud 데이터가 없습니다.');
end

points_A = camera.pointcloud;  % 3×N
fprintf('특징점: %d개 (Chief body frame)\n', size(points_A, 2));

%% 옵티컬 플로우 기반 각속도 추정
fprintf('\n=== 옵티컬 플로우 각속도 추정 ===\n');

n_cam_frames = length(camera.times_s);
omega_optical = struct();
valid_count = 0;

for i = 1:n_cam_frames-1
    t1 = camera.times_s(i);
    t2 = camera.times_s(i+1);
    dt_cam = t2 - t1;
    
    % 측정 픽셀
    z1 = camera.z_measured{i};
    z2 = camera.z_measured{i+1};
    
    if isempty(z1) || isempty(z2)
        continue;
    end
    
    n_points = min(size(z1, 2), size(z2, 2));
    
    if n_points < 3
        continue;
    end
    
    % Ground truth
    [~, idx_t1] = min(abs(time.t_s - t1));
    [~, idx_t2] = min(abs(time.t_s - t2));
    
    omega_true_B = quat.omega_sIeB_observB(:, idx_t1);  % B frame 각속도
    
    %% === t1 시점: A frame → Camera frame 변환 ===
    
    % Chief 상태
    r_A_I_1 = traj.r_A_I(:, idx_t1);
    v_A_I_1 = traj.v_A_I(:, idx_t1);
    r_B_I_1 = traj.r_B_I(:, idx_t1);
    
    % LVLH frame 구성
    r_hat = r_A_I_1 / norm(r_A_I_1);
    h_vec = cross(r_A_I_1, v_A_I_1);
    h_hat = h_vec / norm(h_vec);
    t_hat = cross(h_hat, r_hat);
    R_L2I = [r_hat, t_hat, h_hat];
    R_I2L = R_L2I';
    
    % 자세
    q_L2A = quat.q_L2A(:, idx_t1);
    q_I2B = quat.q_I2B(:, idx_t1);
    
    R_L2A = GetDCM_QUAT(q_L2A);
    R_A2L = R_L2A';
    R_I2B = GetDCM_QUAT(q_I2B);
    
    % 상대 위치 (LVLH)
    r_B_L = traj.r_B_L(:, idx_t1);
    
    % 변환 체인: A → LVLH → B → C
    R_L2B = R_I2B * R_L2I;
    R_L2C = R_B2C * R_L2B;
    
    % 특징점: A → LVLH
    points_L = R_A2L * points_A(:, 1:n_points);  % 3×N
    
    % 특징점: LVLH → Camera (상대 위치 고려)
    points_C_1 = zeros(3, n_points);
    for j = 1:n_points
        points_C_1(:, j) = R_L2C * (points_L(:, j) - r_B_L);
    end
    
    %% === 픽셀 좌표 및 옵티컬 플로우 ===
    uv1 = z1(1:2, 1:n_points);
    uv2 = z2(1:2, 1:n_points);
    uv_dot = (uv2 - uv1) / dt_cam;  % [pixels/sec]
    
    %% === 최소자승법: 순수 회전만 가정 ===
    % u̇ = -ω × (K^-1 · [u; v; 1])
    
    A = zeros(2*n_points, 3);
    b = zeros(2*n_points, 1);
    
    valid_pts = 0;
    
    for j = 1:n_points
        % 깊이 체크
        depth = points_C_1(3, j);
        if depth < 0.01  % 카메라 뒤
            continue;
        end
        
        % 정규화 좌표
        p_h = [uv1(:, j); 1];  % homogeneous
        p_n = K_inv * p_h;      % normalized coords
        
        valid_pts = valid_pts + 1;
        row_idx = 2*(valid_pts - 1) + 1;
        
        % A 행렬: cross product matrix
        % -ω × p_n = [0 p_z -p_y; -p_z 0 p_x; p_y -p_x 0] * ω
        % 처음 두 행만 사용 (u, v만 측정)
        A(row_idx:row_idx+1, :) = [
            0,       p_n(3), -p_n(2);
            -p_n(3), 0,       p_n(1)
        ];
        
        b(row_idx:row_idx+1) = -uv_dot(:, j);
    end
    
    if valid_pts < 3
        continue;
    end
    
    % 실제 사용한 행만
    A = A(1:2*valid_pts, :);
    b = b(1:2*valid_pts);
    
    % 조건수 체크 및 regularization
    [U, S, V] = svd(A' * A);
    s_vals = diag(S);
    cond_num = s_vals(1) / s_vals(end);
    
    if cond_num > 1e12 || any(isnan(s_vals))
        fprintf('  프레임 %d: 수치 불안정 (조건수=%.2e)\n', i, cond_num);
        continue;
    end
    
    if cond_num > 1e8
        % Tikhonov regularization
        lambda = 1e-8 * s_vals(1);
        omega_est_C = (A' * A + lambda * eye(3)) \ (A' * b);
    else
        omega_est_C = (A' * A) \ (A' * b);
    end
    
    % Camera frame → Body frame 변환
    R_C2B = R_B2C';
    omega_est_B = R_C2B * omega_est_C;
    
    % 결과 저장
    valid_count = valid_count + 1;
    omega_optical(valid_count).time = t1;
    omega_optical(valid_count).omega_est_C = omega_est_C;  % Camera frame
    omega_optical(valid_count).omega_est_B = omega_est_B;  % Body frame
    omega_optical(valid_count).omega_true_B = omega_true_B;
    omega_optical(valid_count).n_points = valid_pts;
    omega_optical(valid_count).dt_cam = dt_cam;
    omega_optical(valid_count).cond_num = cond_num;
    
    % 에러
    omega_error = omega_est_B - omega_true_B;
    omega_optical(valid_count).omega_error = omega_error;
    omega_optical(valid_count).omega_error_norm = norm(omega_error);
    
    if mod(valid_count, 5) == 0
        fprintf('  프레임 %d/%d: %d점, 에러=%.4f deg/s, 조건수=%.2e\n', ...
            valid_count, n_cam_frames-1, valid_pts, ...
            norm(omega_error)*180/pi, cond_num);
    end
end

fprintf('완료: %d개 프레임에서 각속도 추정\n', valid_count);

%% 결과 분석
if valid_count > 0
    est_times = [omega_optical.time]' / 60;
    omega_est_B_all = [omega_optical.omega_est_B];
    omega_true_B_all = [omega_optical.omega_true_B];
    omega_error_all = [omega_optical.omega_error];
    omega_error_norm_all = [omega_optical.omega_error_norm];
    cond_nums = [omega_optical.cond_num];
    n_points_all = [omega_optical.n_points];
    
    % 통계
    rms_error = rms(omega_error_norm_all) * 180/pi;
    mean_error = mean(omega_error_norm_all) * 180/pi;
    std_error = std(omega_error_norm_all) * 180/pi;
    max_error = max(omega_error_norm_all) * 180/pi;
    
    fprintf('\n=== 옵티컬 플로우 각속도 추정 성능 ===\n');
    fprintf('RMS 에러:   %.4f deg/s (%.4f deg/h)\n', rms_error, rms_error*3600);
    fprintf('평균 에러:  %.4f deg/s\n', mean_error);
    fprintf('표준편차:   %.4f deg/s\n', std_error);
    fprintf('최대 에러:  %.4f deg/s\n', max_error);
    fprintf('평균 특징점: %.1f개\n', mean(n_points_all));
    fprintf('평균 조건수: %.2e\n', mean(cond_nums));
    
    %% 시각화
    figure('Name', 'Optical Flow Angular Velocity', 'Position', [100, 100, 1400, 900]);
    
    colors = {'r', 'g', 'b'};
    axis_labels = {'\omega_x', '\omega_y', '\omega_z'};
    
    % 3축 각속도
    for ax = 1:3
        subplot(3, 3, ax);
        plot(est_times, omega_est_B_all(ax,:)*180/pi, [colors{ax} '-'], ...
            'LineWidth', 2, 'DisplayName', 'Estimated');
        hold on;
        plot(est_times, omega_true_B_all(ax,:)*180/pi, 'k--', ...
            'LineWidth', 1.5, 'DisplayName', 'Ground Truth');
        xlabel('Time [min]', 'FontSize', 9);
        ylabel('Angular Rate [deg/s]', 'FontSize', 9);
        title([axis_labels{ax} ' (Body Frame)'], 'FontSize', 10);
        legend('Location', 'best', 'FontSize', 8);
        grid on; grid minor;
    end
    
    % 3축 에러
    for ax = 1:3
        subplot(3, 3, 3+ax);
        plot(est_times, omega_error_all(ax,:)*180/pi, colors{ax}, 'LineWidth', 1.5);
        xlabel('Time [min]', 'FontSize', 9);
        ylabel('Error [deg/s]', 'FontSize', 9);
        title([axis_labels{ax} ' Error'], 'FontSize', 10);
        grid on; grid minor;
    end
    
    % 에러 크기
    subplot(3, 3, 7);
    plot(est_times, omega_error_norm_all * 180/pi, 'k-', 'LineWidth', 2);
    xlabel('Time [min]', 'FontSize', 9);
    ylabel('Error Magnitude [deg/s]', 'FontSize', 9);
    title('Total Angular Velocity Error', 'FontSize', 10);
    grid on; grid minor;
    
    % 조건수
    subplot(3, 3, 8);
    semilogy(est_times, cond_nums, 'b-', 'LineWidth', 1.5);
    xlabel('Time [min]', 'FontSize', 9);
    ylabel('Condition Number', 'FontSize', 9);
    title('Matrix Conditioning', 'FontSize', 10);
    grid on;
    
    % 통계 정보
    subplot(3, 3, 9);
    axis off;
    text(0.1, 0.85, '성능 요약', 'FontSize', 11, 'FontWeight', 'bold');
    text(0.1, 0.70, sprintf('RMS: %.4f deg/s', rms_error), 'FontSize', 10);
    text(0.1, 0.58, sprintf('평균: %.4f deg/s', mean_error), 'FontSize', 10);
    text(0.1, 0.46, sprintf('표준편차: %.4f deg/s', std_error), 'FontSize', 10);
    text(0.1, 0.34, sprintf('최대: %.4f deg/s', max_error), 'FontSize', 10);
    text(0.1, 0.22, sprintf('측정: %d회', valid_count), 'FontSize', 10);
    text(0.1, 0.10, sprintf('평균 점: %.0f개', mean(n_points_all)), 'FontSize', 10);
    
    sgtitle('Optical Flow-based Angular Velocity Estimation', ...
        'FontSize', 14, 'FontWeight', 'bold');
    
    %% IMU 자이로와 비교
    figure('Name', 'Optical Flow vs IMU Gyro', 'Position', [150, 150, 1400, 600]);
    
    gyro_error_norm = vecnorm(imu.gyro_meas - imu.gyro_true, 2, 1) * 180/pi;
    
    % 시계열 비교
    subplot(1, 3, 1);
    plot(time.t_min, gyro_error_norm, 'b-', 'LineWidth', 1, 'DisplayName', 'IMU Gyro (5Hz)');
    hold on;
    plot(est_times, omega_error_norm_all * 180/pi, 'ro-', ...
        'LineWidth', 2, 'MarkerSize', 6, 'DisplayName', 'Optical Flow (1/60Hz)');
    xlabel('Time [min]', 'FontSize', 10);
    ylabel('Error Magnitude [deg/s]', 'FontSize', 10);
    title('Angular Velocity Error Comparison', 'FontSize', 11);
    legend('Location', 'best', 'FontSize', 9);
    grid on; grid minor;
    
    % RMS 비교
    subplot(1, 3, 2);
    bar([rms(gyro_error_norm), rms_error]);
    set(gca, 'XTickLabel', {'IMU Gyro', 'Optical Flow'});
    ylabel('RMS Error [deg/s]', 'FontSize', 10);
    title('RMS Error Comparison', 'FontSize', 11);
    grid on;
    
    % 에러 분포
    subplot(1, 3, 3);
    histogram(gyro_error_norm, 50, 'FaceColor', 'b', 'FaceAlpha', 0.5, ...
        'DisplayName', 'IMU Gyro');
    hold on;
    histogram(omega_error_norm_all * 180/pi, 20, 'FaceColor', 'r', 'FaceAlpha', 0.5, ...
        'DisplayName', 'Optical Flow');
    xlabel('Error [deg/s]', 'FontSize', 10);
    ylabel('Count', 'FontSize', 10);
    title('Error Distribution', 'FontSize', 11);
    legend('Location', 'best', 'FontSize', 9);
    grid on;
    
    sgtitle('Sensor Performance Comparison', 'FontSize', 14, 'FontWeight', 'bold');
    
    fprintf('\n=== 센서 비교 ===\n');
    fprintf('IMU 자이로 RMS:      %.6f deg/s\n', rms(gyro_error_norm));
    fprintf('옵티컬 플로우 RMS:   %.6f deg/s\n', rms_error);
    fprintf('비율 (OF/IMU):       %.2fx\n', rms_error / rms(gyro_error_norm));
    
    %% 결과 저장
    % output_file = fullfile(data_dir, sprintf('optical_flow_results_%s.mat', ...
    %     datestr(now, 'yyyymmdd_HHMMSS')));
    % save(output_file, 'omega_optical', 'K', 'R_B2C', 'meta', ...
    %     'rms_error', 'mean_error', 'std_error', 'max_error', '-v7.3');
    % fprintf('\n결과 저장: %s\n', output_file);
    
else
    warning('유효한 추정 결과가 없습니다.');
end

fprintf('\n=== 옵티컬 플로우 분석 완료 ===\n');