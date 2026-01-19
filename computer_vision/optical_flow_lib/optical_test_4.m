clc; clear all; close all;
addpath(genpath('C:\Users\USER\Desktop\relative2'));

%% 데이터 로드
data_dir = fullfile(pwd, 'sim_data');
files = dir(fullfile(data_dir, 'sim_data_ver2_*.mat'));
latest_file = fullfile(data_dir, files(end).name);
load(latest_file);

fprintf('=== 6-DOF 옵티컬 플로우 (상대 운동 측정) ===\n\n');

%% 카메라 파라미터
K = camera.params.intrinsic.K_ideal;
R_B2C = [1, 0, 0; 0, 0, -1; 0, 1, 0];
R_C2B = R_B2C';

fx = K(1,1);
fy = K(2,2);
cx = K(1,3);
cy = K(2,3);

fprintf('카메라: fx=%.2f, fy=%.2f\n', fx, fy);

%% Point Cloud
points_A = camera.pointcloud;
fprintf('특징점: %d개\n\n', size(points_A, 2));

%% 6-DOF 옵티컬 플로우 추정
n_cam = length(camera.times_s);
results = struct();
valid_count = 0;

for i = 1:n_cam-1
    t1 = camera.times_s(i);
    t2 = camera.times_s(i+1);
    dt = t2 - t1;
    
    z1 = camera.z_measured{i};
    z2 = camera.z_measured{i+1};
    
    if isempty(z1) || isempty(z2)
        continue;
    end
    
    n_pts = min(size(z1, 2), size(z2, 2));
    if n_pts < 6; continue; end
    
    % 시간 인덱스
    [~, idx1] = min(abs(time.t_s - t1));
    [~, idx2] = min(abs(time.t_s - t2));
    idx_mid = round((idx1 + idx2) / 2);
    
    %% === Ground Truth: 상대 운동 ===
    % ω^B_{A/B}: B 기준 A가 회전, B frame 관측
    omega_sBeA_observB_true = quat.omega_sBeA_observB(:, idx1);
    
    % v^L_{A/B}: B 기준 A가 병진, LVLH frame 관측
    % = -v^L_{B/A} = -(v_B - v_A) in LVLH
    v_sLeB_observL_true = traj.v_B_L(:, idx1);  % ✅ 해석적 속도
    v_sBeA_observL_true = -v_sLeB_observL_true;
    
    %% 좌표 변환 체인 (중간 시점 사용)
    r_A_I = traj.r_A_I(:, idx_mid);
    v_A_I = traj.v_A_I(:, idx_mid);
    
    r_hat = r_A_I / norm(r_A_I);
    h_vec = cross(r_A_I, v_A_I);
    h_hat = h_vec / norm(h_vec);
    t_hat = cross(h_hat, r_hat);
    R_L2I = [r_hat, t_hat, h_hat];
    R_I2L = R_L2I';
    
    q_L2A = quat.q_L2A(:, idx_mid);
    q_I2B = quat.q_I2B(:, idx_mid);
    
    R_L2A = GetDCM_QUAT(q_L2A);
    R_A2L = R_L2A';
    R_I2B = GetDCM_QUAT(q_I2B);
    R_L2B = R_I2B * R_L2I;
    R_L2C = R_B2C * R_L2B;
    R_C2L = R_L2C';
    
    % 특징점: A → LVLH → Camera (중간 시점 위치)
    r_B_L_mid = (traj.r_B_L(:, idx1) + traj.r_B_L(:, idx2)) / 2;
    points_L = R_A2L * points_A(:, 1:n_pts);
    points_C = zeros(3, n_pts);
    for j = 1:n_pts
        points_C(:, j) = R_L2C * (points_L(:, j) - r_B_L_mid);
    end
    
    %% 옵티컬 플로우
    uv1 = z1(1:2, 1:n_pts);
    uv2 = z2(1:2, 1:n_pts);
    uv_dot = (uv2 - uv1) / dt;  % [px/s]
    
    %% 6-DOF 최소자승
    % x = [ω_C; v_C]  (6×1, Camera frame)
    % ω_C: ω^C_{A/B} (B 기준 A가 회전, C frame 관측)
    % v_C: v^C_{A/B} (B 기준 A가 병진, C frame 관측)
    
    A = zeros(2*n_pts, 6);
    b = zeros(2*n_pts, 1);
    
    valid = 0;
    
    for j = 1:n_pts
        P_C = points_C(:, j);
        depth = P_C(3);
        
        if depth < 0.01; continue; end
        
        X = P_C(1);
        Y = P_C(2);
        Z = P_C(3);
        
        % 정규화 좌표
        x_n = X / Z;
        y_n = Y / Z;
        
        % 옵티컬 플로우 자코비안
        % 회전 항: -ω × [x_n; y_n; 1]의 x,y 성분
        J_rot_u = fx * [x_n*y_n, -(1+x_n^2), y_n];
        J_rot_v = fy * [1+y_n^2, -x_n*y_n, -x_n];
        
        % 병진 항: -(1/Z)[I - p_n * p_n']의 x,y row
        % A가 B로부터 멀어질 때 픽셀 감소 → 음수 부호
        J_trans_u = -(fx/Z) * [1, 0, -x_n];
        J_trans_v = -(fy/Z) * [0, 1, -y_n];
        
        valid = valid + 1;
        row = 2*(valid-1) + 1;
        
        A(row,   1:3) = J_rot_u;
        A(row,   4:6) = J_trans_u;
        A(row+1, 1:3) = J_rot_v;
        A(row+1, 4:6) = J_trans_v;
        
        b(row:row+1) = uv_dot(:, j);
    end
    
    if valid < 6; continue; end
    
    A = A(1:2*valid, :);
    b = b(1:2*valid);
    
    % SVD 체크
    [U, S, V] = svd(A' * A);
    s = diag(S);
    cond_num = s(1) / s(end);
    
    if cond_num > 1e12 || any(isnan(s))
        fprintf('  프레임 %d: 수치 불안정 (cond=%.2e)\n', i, cond_num);
        continue;
    end
    
    % 최소자승 해
    if cond_num > 1e8
        lambda = 1e-8 * s(1);
        x_est = (A' * A + lambda * eye(6)) \ (A' * b);
    else
        x_est = (A' * A) \ (A' * b);
    end
    
    % 추정값 (Camera frame)
    omega_sBeA_observC_est = x_est(1:3);
    v_sBeA_observC_est = x_est(4:6);
    
    % Camera → Body/LVLH frame 변환
    omega_sBeA_observB_est = R_C2B * omega_sBeA_observC_est;
    v_sBeA_observL_est = R_C2L * v_sBeA_observC_est;
    
    % Ground truth (Camera frame)
    omega_sBeA_observC_true = R_B2C * omega_sBeA_observB_true;
    v_sBeA_observC_true = R_L2C * v_sBeA_observL_true;
    
    % 저장
    valid_count = valid_count + 1;
    results(valid_count).time = t1;
    results(valid_count).omega_sBeA_observB_est = omega_sBeA_observB_est;
    results(valid_count).omega_sBeA_observB_true = omega_sBeA_observB_true;
    results(valid_count).v_sBeA_observL_est = v_sBeA_observL_est;
    results(valid_count).v_sBeA_observL_true = v_sBeA_observL_true;
    results(valid_count).omega_error = omega_sBeA_observB_est - omega_sBeA_observB_true;
    results(valid_count).v_error = v_sBeA_observL_est - v_sBeA_observL_true;
    results(valid_count).omega_error_norm = norm(omega_sBeA_observB_est - omega_sBeA_observB_true);
    results(valid_count).v_error_norm = norm(v_sBeA_observL_est - v_sBeA_observL_true);
    results(valid_count).n_points = valid;
    results(valid_count).cond_num = cond_num;
    
    % 첫 프레임 검증 출력
    if valid_count == 1
        fprintf('=== 첫 프레임 검증 ===\n');
        fprintf('GT 병진속도 (LVLH): [%.6f, %.6f, %.6f] m/s\n', ...
            v_sBeA_observL_true*1000);
        fprintf('Est 병진속도 (LVLH): [%.6f, %.6f, %.6f] m/s\n', ...
            v_sBeA_observL_est*1000);
        fprintf('GT 각속도 (Body):   [%.6f, %.6f, %.6f] deg/s\n', ...
            omega_sBeA_observB_true*180/pi);
        fprintf('Est 각속도 (Body):  [%.6f, %.6f, %.6f] deg/s\n\n', ...
            omega_sBeA_observB_est*180/pi);
    end
    
    if mod(valid_count, 5) == 0
        fprintf('  프레임 %d: ω_err=%.4f deg/s, v_err=%.4f m/s, cond=%.2e\n', ...
            valid_count, results(valid_count).omega_error_norm*180/pi, ...
            results(valid_count).v_error_norm*1000, cond_num);
    end

    %% === 진단 코드 (첫 프레임만) ===
    if i == 1
        fprintf('\n=== 진단: 첫 프레임 상세 분석 ===\n');
        
        % 1. 깊이 분포
        depths_valid = points_C(3, points_C(3,:) > 0.01);
        fprintf('깊이: min=%.3f, max=%.3f, mean=%.3f km\n', ...
            min(depths_valid), max(depths_valid), mean(depths_valid));
        
        % 2. 픽셀 속도 크기
        uv_speed = vecnorm(uv_dot, 2, 1);
        fprintf('픽셀 속도: mean=%.2f, max=%.2f px/s\n', ...
            mean(uv_speed), max(uv_speed));
        
        % 3. Ground Truth (Camera frame)
        fprintf('\nGT in Camera frame:\n');
        fprintf('  ω^C_{A/B}: [%.6f, %.6f, %.6f] deg/s\n', ...
            omega_sBeA_observC_true * 180/pi);
        fprintf('  v^C_{A/B}: [%.6f, %.6f, %.6f] m/s\n', ...
            v_sBeA_observC_true * 1000);
        
        % 4. 자코비안 샘플 (첫 3개 포인트)
        fprintf('\n자코비안 샘플 (첫 3개 특징점):\n');
        for jj = 1:min(3, valid)
            row = 2*(jj-1) + 1;
            fprintf('  P%d: depth=%.3f, J_trans_u=[%.2e, %.2e, %.2e]\n', ...
                jj, points_C(3, jj), A(row, 4), A(row, 5), A(row, 6));
        end
        
        % 5. A의 조건수와 singular values
        fprintf('\n행렬 A: cond=%.2e, size=%dx%d\n', cond_num, size(A,1), size(A,2));
        fprintf('  Singular values: [%.2e, %.2e, %.2e, %.2e, %.2e, %.2e]\n', s);
        
        % 6. 이론적 픽셀 속도 계산 (GT 사용)
        uv_dot_theory = zeros(2, n_pts);
        for j = 1:n_pts
            P_C = points_C(:, j);
            if P_C(3) < 0.01; continue; end
            
            X = P_C(1); Y = P_C(2); Z = P_C(3);
            x_n = X/Z; y_n = Y/Z;
            
            % GT 사용
            omega_C = omega_sBeA_observC_true;
            v_C = v_sBeA_observC_true;
            
            % 이론적 픽셀 속도
            u_dot_th = -(fx/Z)*v_C(1) + (fx*x_n/Z)*v_C(3) + ...
                       fx*(x_n*y_n*omega_C(1) - (1+x_n^2)*omega_C(2) + y_n*omega_C(3));
            v_dot_th = -(fy/Z)*v_C(2) + (fy*y_n/Z)*v_C(3) + ...
                       fy*((1+y_n^2)*omega_C(1) - x_n*y_n*omega_C(2) - x_n*omega_C(3));
            
            uv_dot_theory(:, j) = [u_dot_th; v_dot_th];
        end
        
        % 7. 실측 vs 이론 픽셀 속도 비교
        uv_diff = uv_dot - uv_dot_theory;
        uv_diff_norm = vecnorm(uv_diff, 2, 1);
        fprintf('\n픽셀 속도 오차 (측정-이론):\n');
        fprintf('  mean=%.4f, max=%.4f, std=%.4f px/s\n', ...
            mean(uv_diff_norm), max(uv_diff_norm), std(uv_diff_norm));
        
        % 8. GT로 직접 추정 (이론적 하한)
        x_gt_C = [omega_sBeA_observC_true; v_sBeA_observC_true];
        residual_gt = A * x_gt_C - b;
        fprintf('\nGT 사용 시 residual norm: %.4e\n', norm(residual_gt));
        
        fprintf('=== 진단 종료 ===\n\n');
        
        % 디버깅 플롯
        figure('Position', [100, 100, 1200, 400]);
        
        subplot(1, 3, 1);
        histogram(depths_valid, 30);
        xlabel('Depth [km]'); ylabel('Count');
        title('특징점 깊이 분포');
        grid on;
        
        subplot(1, 3, 2);
        histogram(uv_speed, 30);
        xlabel('Pixel Speed [px/s]'); ylabel('Count');
        title('픽셀 속도 크기');
        grid on;
        
        subplot(1, 3, 3);
        scatter(uv_dot(1,:), uv_dot(2,:), 10, 'filled');
        hold on;
        scatter(uv_dot_theory(1,:), uv_dot_theory(2,:), 10, 'r', 'filled');
        xlabel('u\_dot [px/s]'); ylabel('v\_dot [px/s]');
        title('픽셀 속도 (측정 vs 이론)');
        legend('측정', 'GT 이론', 'Location', 'best');
        axis equal; grid on;
    end
end

fprintf('완료: %d개 프레임\n\n', valid_count);

%% 결과 분석
if valid_count > 0
    times = [results.time]' / 60;
    
    omega_est = [results.omega_sBeA_observB_est];
    omega_true = [results.omega_sBeA_observB_true];
    omega_err = [results.omega_error];
    omega_err_norm = [results.omega_error_norm] * 180/pi;
    
    v_est = [results.v_sBeA_observL_est];
    v_true = [results.v_sBeA_observL_true];
    v_err = [results.v_error];
    v_err_norm = [results.v_error_norm] * 1000;  % m/s
    
    fprintf('=== 각속도 추정 성능 (ω^B_{A/B}: 상대 각속도) ===\n');
    fprintf('RMS:  %.4f deg/s\n', rms(omega_err_norm));
    fprintf('평균: %.4f deg/s\n', mean(omega_err_norm));
    fprintf('최대: %.4f deg/s\n', max(omega_err_norm));
    fprintf('STD:  %.4f deg/s\n', std(omega_err_norm));
    
    fprintf('\n=== 병진속도 추정 성능 (v^L_{A/B}: 상대 병진속도) ===\n');
    fprintf('RMS:  %.4f m/s\n', rms(v_err_norm));
    fprintf('평균: %.4f m/s\n', mean(v_err_norm));
    fprintf('최대: %.4f m/s\n', max(v_err_norm));
    fprintf('STD:  %.4f m/s\n', std(v_err_norm));
    
    %% 시각화
    figure('Position', [100, 100, 1600, 1000]);
    
    % 상대 각속도 (Body frame)
    for ax = 1:3
        subplot(4, 3, ax);
        plot(times, omega_est(ax,:)*180/pi, 'r-', 'LineWidth', 2);
        hold on;
        plot(times, omega_true(ax,:)*180/pi, 'k--', 'LineWidth', 1.5);
        ylabel('[deg/s]'); grid on;
        title(sprintf('\\omega^B_{A/B,%s}', char('x'+ax-1)));
        if ax==1; legend('Est', 'True', 'Location', 'best'); end
    end
    
    % 각속도 에러
    for ax = 1:3
        subplot(4, 3, 3+ax);
        plot(times, omega_err(ax,:)*180/pi, 'LineWidth', 1.5);
        ylabel('[deg/s]'); grid on;
        title(sprintf('\\omega Error (%s)', char('x'+ax-1)));
    end
    
    % 상대 병진속도 (LVLH frame)
    for ax = 1:3
        subplot(4, 3, 6+ax);
        plot(times, v_est(ax,:)*1000, 'b-', 'LineWidth', 2);
        hold on;
        plot(times, v_true(ax,:)*1000, 'k--', 'LineWidth', 1.5);
        ylabel('[m/s]'); grid on;
        title(sprintf('v^L_{A/B,%s}', char('x'+ax-1)));
        if ax==1; legend('Est', 'True', 'Location', 'best'); end
    end
    
    % 병진속도 에러
    for ax = 1:3
        subplot(4, 3, 9+ax);
        plot(times, v_err(ax,:)*1000, 'LineWidth', 1.5);
        xlabel('Time [min]'); ylabel('[m/s]'); grid on;
        title(sprintf('v Error (%s)', char('x'+ax-1)));
    end
    
    sgtitle('6-DOF Optical Flow: Relative Motion Estimation', 'FontSize', 14, 'FontWeight', 'bold');
    
    %% 비교
    figure('Position', [150, 150, 1400, 500]);
    
    subplot(1, 3, 1);
    plot(times, omega_err_norm, 'r-', 'LineWidth', 2);
    ylabel('Error [deg/s]'); xlabel('Time [min]');
    title('\omega^B_{A/B} Error (Relative Angular Velocity)'); grid on;
    
    subplot(1, 3, 2);
    plot(times, v_err_norm, 'b-', 'LineWidth', 2);
    ylabel('Error [m/s]'); xlabel('Time [min]');
    title('v^L_{A/B} Error (Relative Translational Velocity)'); grid on;
    
    subplot(1, 3, 3);
    histogram(omega_err_norm, 30);
    xlabel('Angular Velocity Error [deg/s]'); ylabel('Count');
    title(sprintf('Error Distribution (RMS: %.4f deg/s)', rms(omega_err_norm)));
    grid on;
    
    % Ground Truth 통계
    omega_rel_true_mag = vecnorm(omega_true, 2, 1) * 180/pi;
    v_rel_true_mag = vecnorm(v_true, 2, 1) * 1000;
    
    fprintf('\n=== Ground Truth 통계 ===\n');
    fprintf('상대 각속도 크기: %.6f deg/s (평균)\n', mean(omega_rel_true_mag));
    fprintf('상대 병진속도 크기: %.6f m/s (평균)\n', mean(v_rel_true_mag));
    fprintf('\n=== 상대 오차 (RMS/True) ===\n');
    fprintf('각속도: %.2f%%\n', 100*rms(omega_err_norm)/mean(omega_rel_true_mag));
    fprintf('병진속도: %.2f%%\n', 100*rms(v_err_norm)/mean(v_rel_true_mag));
    
else
    warning('추정 실패');
end

fprintf('\n완료.\n');