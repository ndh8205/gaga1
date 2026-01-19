%% debug_08_paper_with_noise.m
% Paper 버전 테스트 (노이즈 포함 데이터 사용)
clear; close all; clc;
addpath(genpath('D:\star_tracker_test\main_pj_code'));

fprintf('=== Paper 버전 테스트 (노이즈 데이터) ===\n\n');

%% 1. 카탈로그 로드
tic;
load('star_catalog_fast.mat');
t_load = toc;
fprintf('카탈로그 로드: %.2f ms\n\n', t_load*1000);

%% 2. 센서 파라미터
sensor_params.f = 0.01042;
sensor_params.myu = 2e-6;
sensor_params.l = 1280;
sensor_params.w = 720;
sensor_params.sigma = 50e-6;
sensor_params.k_multiplier = 6.4;

%% 3. 데이터 로드
data_dir = fullfile(pwd, 'sim_data');
star_data_noise_dir = fullfile(data_dir, 'star_data', 'noise');

% sim_data 로드
sim_files = dir(fullfile(data_dir, 'sim_data_ver2_*.mat'));
if isempty(sim_files)
    error('sim_data 파일 없음');
end
[~, latest_sim_idx] = max([sim_files.datenum]);
load(fullfile(data_dir, sim_files(latest_sim_idx).name));
fprintf('로드 (sim): %s\n', sim_files(latest_sim_idx).name);

% 노이즈 포함 star_data 로드
star_files = dir(fullfile(star_data_noise_dir, 'star_data_noise*.mat'));
if isempty(star_files)
    error('노이즈 데이터 없음');
end
[~, latest_star_idx] = max([star_files.datenum]);
load(fullfile(star_data_noise_dir, star_files(latest_star_idx).name));
fprintf('로드 (star): %s\n', star_files(latest_star_idx).name);
fprintf('픽셀 노이즈: %.2f pixels\n', sigma_pixel);
fprintf('총 프레임: %d\n\n', length(star_data_noise_all));

%% 4. 테스트 프레임 선택
num_stars = [star_data_noise_all.num_stars];
[~, sorted_idx] = sort(num_stars, 'descend');

n_test_frames = min(10, sum(num_stars >= 4));
test_frames = sorted_idx(1:n_test_frames);

fprintf('테스트 프레임: %d개 (별 >= 4개)\n\n', n_test_frames);

%% 5. 통계 변수
success_count = 0;
attitude_errors = [];
execution_times = [];
matching_accuracies = [];

results = struct('frame', {}, 'n_stars', {}, 'success', {}, ...
                 'n_identified', {}, 'att_error', {}, 'exec_time', {}, 'matching_acc', {});

%% 6. 각 프레임 테스트
R_B2ST = [1, 0, 0; 0, 0, 1; 0,-1, 0];
R_ST2B = R_B2ST';

for test_idx = 1:n_test_frames
    k = test_frames(test_idx);
    frame_data = star_data_noise_all(k);
    
    fprintf('--- Frame %d/%d (총 별: %d개) ---\n', test_idx, n_test_frames, frame_data.num_stars);
    
    % 노이즈 이미 포함된 데이터 사용
    pixel_coords = frame_data.pixel_coords;
    magnitudes = frame_data.magnitudes;
    frame_idx = frame_data.frame;
    
    % Ground truth 자세
    q_true = quat.q_I2B(:, frame_idx);
    R_I2B_true = GetDCM_QUAT(q_true);
    
    % Star-ID 실행
    tic;
    [star_ids, attitude, pyramid_stats] = pyramid_star_id_liebe2002(...
        pixel_coords, magnitudes, catalog_fast, sensor_params);
    exec_time = toc;
    
    % 결과 저장
    results(test_idx).frame = k;
    results(test_idx).n_stars = frame_data.num_stars;
    results(test_idx).success = pyramid_stats.success;
    results(test_idx).exec_time = exec_time;
    
    if pyramid_stats.success
        success_count = success_count + 1;
        execution_times = [execution_times; exec_time];
        results(test_idx).n_identified = pyramid_stats.n_identified;
        
        % ★ Ground Truth 매칭 확인
        true_ids = frame_data.star_ids;
        est_ids = star_ids(:)';  % row vector
        
        n_correct = sum(ismember(est_ids, true_ids));
        n_wrong = length(est_ids) - n_correct;
        matching_acc = 100 * n_correct / length(est_ids);
        
        fprintf('    [MATCH] %d correct, %d wrong (%.1f%% accuracy)\n', ...
            n_correct, n_wrong, matching_acc);
        
        if n_wrong > 0
            wrong_mask = ~ismember(est_ids, true_ids);
            wrong_ids = est_ids(wrong_mask);
            fprintf('    [WRONG] False IDs: ');
            fprintf('%d ', wrong_ids);
            fprintf('\n');
        end
        
        matching_accuracies = [matching_accuracies; matching_acc];
        
        % 자세 오차
        if ~isempty(attitude)
            R_I2ST_est = attitude.R;
            R_I2B_est = R_ST2B * R_I2ST_est;
            
            % ★ 디버그: 행렬 출력 (첫 프레임만)
            if test_idx == 1
                fprintf('\n    [DEBUG] R_I2B_est:\n');
                fprintf('      [%.6f %.6f %.6f]\n', R_I2B_est(1,:));
                fprintf('      [%.6f %.6f %.6f]\n', R_I2B_est(2,:));
                fprintf('      [%.6f %.6f %.6f]\n', R_I2B_est(3,:));
                
                fprintf('    [DEBUG] R_I2B_true:\n');
                fprintf('      [%.6f %.6f %.6f]\n', R_I2B_true(1,:));
                fprintf('      [%.6f %.6f %.6f]\n', R_I2B_true(2,:));
                fprintf('      [%.6f %.6f %.6f]\n', R_I2B_true(3,:));
            end
            
            R_err = R_I2B_est * R_I2B_true';
            
            if test_idx == 1
                fprintf('    [DEBUG] R_err:\n');
                fprintf('      [%.6f %.6f %.6f]\n', R_err(1,:));
                fprintf('      [%.6f %.6f %.6f]\n', R_err(2,:));
                fprintf('      [%.6f %.6f %.6f]\n', R_err(3,:));
            end
            
            q_err = DCM2Quat(R_err);
            
            if test_idx == 1
                fprintf('    [DEBUG] q_err = [%.10f %.10f %.10f %.10f]\n', q_err);
            end
            
            % ★★★ 정밀 계산 (벡터 부분 사용) ★★★
            qv_norm = norm(q_err(2:4));
            
            if qv_norm < 0.01
                % 작은 각도: θ ≈ 2*||qv||
                angle_err = 2 * qv_norm;
            else
                % 큰 각도
                q_scalar = max(-1, min(1, q_err(1)));
                angle_err = 2 * acos(abs(q_scalar));
            end
            
            angle_err_arcsec = angle_err * 206265;
            
            if test_idx == 1
                fprintf('    [DEBUG] qv_norm = %.10f\n', qv_norm);
                fprintf('    [DEBUG] angle_err = %.10f rad = %.6f arcsec\n\n', angle_err, angle_err_arcsec);
            end
            
            % 예상 오차 (이론값)
            angle_per_pixel = atan(sensor_params.myu / sensor_params.f);
            expected_err = sigma_pixel * angle_per_pixel / sqrt(n_correct) * 206265;
            
            fprintf('    [ATT] Measured: %.2f", Expected: %.2f" (√%d reduction)\n', ...
                angle_err_arcsec, expected_err, n_correct);
            
            results(test_idx).att_error = angle_err_arcsec;
            results(test_idx).matching_acc = matching_acc;
            attitude_errors = [attitude_errors; angle_err_arcsec];
            
            fprintf('  ✓ 성공: %d개 (%.1f%% 정확), 자세 %.2f arcsec, %.2f ms\n', ...
                pyramid_stats.n_identified, matching_acc, angle_err_arcsec, exec_time*1000);
        else
            results(test_idx).att_error = NaN;
            results(test_idx).matching_acc = matching_acc;
            fprintf('  ✓ 식별 성공: %d개 (%.1f%% 정확), %.2f ms\n', ...
                pyramid_stats.n_identified, matching_acc, exec_time*1000);
        end
    else
        results(test_idx).n_identified = 0;
        results(test_idx).att_error = NaN;
        results(test_idx).matching_acc = 0;
        fprintf('  ✗ 실패: %.2f ms\n', exec_time*1000);
    end
end

%% 7. 통계 요약
fprintf('\n=== 전체 통계 (노이즈: %.2f pixels) ===\n', sigma_pixel);
fprintf('성공률: %d/%d (%.1f%%)\n', success_count, n_test_frames, 100*success_count/n_test_frames);

if ~isempty(matching_accuracies)
    fprintf('매칭 정확도: %.1f ± %.1f%%\n', mean(matching_accuracies), std(matching_accuracies));
end

if ~isempty(execution_times)
    fprintf('실행시간: %.2f ± %.2f ms\n', mean(execution_times)*1000, std(execution_times)*1000);
end

if ~isempty(attitude_errors)
    fprintf('자세 오차: %.2f ± %.2f arcsec (%.2f ~ %.2f arcsec)\n', ...
        mean(attitude_errors), std(attitude_errors), ...
        min(attitude_errors), max(attitude_errors));
    
    high_precision = sum(attitude_errors < 6);
    medium_precision = sum(attitude_errors >= 6 & attitude_errors < 60);
    low_precision = sum(attitude_errors >= 60);
    
    fprintf('\n자세 정밀도 분포:\n');
    fprintf('  고정밀 (<6"): %d (%.1f%%)\n', high_precision, 100*high_precision/length(attitude_errors));
    fprintf('  중정밀 (6~60"): %d (%.1f%%)\n', medium_precision, 100*medium_precision/length(attitude_errors));
    fprintf('  저정밀 (>60"): %d (%.1f%%)\n', low_precision, 100*low_precision/length(attitude_errors));
end

%% 8. 시각화
if success_count > 0
    figure('Name', 'Paper Version Performance', 'Position', [100, 100, 1400, 800]);
    
    subplot(2,3,1);
    if ~isempty(attitude_errors)
        histogram(attitude_errors, 20, 'FaceColor', [0.2 0.6 0.8]);
        grid on;
        xlabel('Attitude Error (arcsec)');
        ylabel('Count');
        title(sprintf('Attitude Error (mean: %.2f")', mean(attitude_errors)));
    end
    
    subplot(2,3,2);
    histogram(execution_times*1000, 20, 'FaceColor', [0.8 0.4 0.2]);
    grid on;
    xlabel('Execution Time (ms)');
    ylabel('Count');
    title(sprintf('Execution Time (mean: %.2f ms)', mean(execution_times)*1000));
    
    subplot(2,3,3);
    n_identified = [results([results.success]).n_identified];
    histogram(n_identified, 'BinMethod', 'integers', 'FaceColor', [0.4 0.8 0.4]);
    grid on;
    xlabel('Stars Identified');
    ylabel('Count');
    title(sprintf('Identification (mean: %.1f)', mean(n_identified)));
    
    subplot(2,3,4);
    if ~isempty(attitude_errors)
        n_stars_success = [results([results.success]).n_stars];
        scatter(n_stars_success, attitude_errors, 50, 'filled');
        grid on;
        xlabel('Number of Stars');
        ylabel('Attitude Error (arcsec)');
        title('Error vs Star Count');
    end
    
    subplot(2,3,5);
    scatter(n_identified, attitude_errors, 50, 'filled');
    grid on;
    xlabel('Stars Identified');
    ylabel('Attitude Error (arcsec)');
    title('Error vs Identified Stars');
    
    subplot(2,3,6);
    if ~isempty(matching_accuracies)
        histogram(matching_accuracies, 20, 'FaceColor', [0.6 0.4 0.8]);
        grid on;
        xlabel('Matching Accuracy (%)');
        ylabel('Count');
        title(sprintf('Matching (mean: %.1f%%)', mean(matching_accuracies)));
    end
end

fprintf('\n=== 테스트 완료 ===\n');