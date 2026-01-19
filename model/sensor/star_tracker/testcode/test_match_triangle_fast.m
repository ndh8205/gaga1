%% test_match_triangle_fast.m
% match_triangle_fast 함수 검증
clear; close all; clc;

fprintf('=== match_triangle_fast 테스트 ===\n\n');

%% 1. 데이터 로드
data_dir = fullfile(pwd, 'sim_data');
star_data_noise_dir = fullfile(data_dir, 'star_data', 'noise');

% 시뮬레이션 데이터
sim_files = dir(fullfile(data_dir, 'sim_data_lqr_*.mat'));
[~, latest_sim_idx] = max([sim_files.datenum]);
load(fullfile(data_dir, sim_files(latest_sim_idx).name));

% 별 데이터
star_files = dir(fullfile(star_data_noise_dir, 'star_data_noise*.mat'));
[~, latest_star_idx] = max([star_files.datenum]);
load(fullfile(star_data_noise_dir, star_files(latest_star_idx).name));

% 카탈로그
load('star_catalog_fast.mat');

fprintf('[로드 완료]\n');
fprintf('  총 프레임: %d\n', length(star_data_noise_all));
fprintf('  카탈로그: %d stars\n\n', catalog_fast.N_stars);

%% 2. 센서 파라미터
sensor_params.f = 0.01042;
sensor_params.myu = 2e-6;
sensor_params.l = 1280;
sensor_params.w = 720;
sensor_params.sigma = 50e-6;
sensor_params.k_multiplier = 6.4;

%% 3. 테스트 프레임 선택
n_test_frames = min(100, length(star_data_noise_all));
test_frames = randsample(length(star_data_noise_all), n_test_frames);

fprintf('[테스트 설정]\n');
fprintf('  테스트 프레임: %d개\n', n_test_frames);
fprintf('  Triangle/프레임: 최대 5개\n\n');

%% 4. 통계 변수 초기화
stats.total_triangles = 0;
stats.true_positive = 0;
stats.false_positive = 0;
stats.false_negative = 0;
stats.execution_times = [];
stats.failed_cases = {};

%% 5. 프레임별 테스트
fprintf('=== 테스트 진행 ===\n');
for f_idx = 1:n_test_frames
    frame = test_frames(f_idx);
    frame_data = star_data_noise_all(frame);
    
    if mod(f_idx, 20) == 0
        fprintf('  진행: %d/%d\n', f_idx, n_test_frames);
    end
    
    % 별 3개 미만은 스킵
    if frame_data.num_stars < 3
        continue;
    end
    
    % 픽셀 → 단위벡터
    pixel_coords = frame_data.pixel_coords;
    magnitudes = frame_data.magnitudes;
    n = size(pixel_coords, 1);
    
    b_vectors = pixel_to_unit_vector(pixel_coords, sensor_params.f, ...
        sensor_params.myu, sensor_params.l, sensor_params.w);
    
    % 각거리 행렬
    angle_matrix = precompute_angle_matrix(b_vectors);
    
    % Triangle 선택
    triangle_list = smart_triangle_selection(n, magnitudes, angle_matrix, 5);
    
    % 각 triangle 테스트
    for t = 1:size(triangle_list, 1)
        i = triangle_list(t, 1);
        j = triangle_list(t, 2);
        k = triangle_list(t, 3);
        
        stats.total_triangles = stats.total_triangles + 1;
        
        % Ground truth
        gt_star_ids = frame_data.star_ids([i, j, k]);
        gt_cat_idx = zeros(1, 3);
        for m = 1:3
            idx_find = find(catalog_fast.star_ID == gt_star_ids(m), 1);
            if isempty(idx_find)
                gt_cat_idx(m) = -1;  % 카탈로그 없음
            else
                gt_cat_idx(m) = idx_find;
            end
        end
        
        % 카탈로그에 없는 별 포함 시 스킵
        if any(gt_cat_idx == -1)
            continue;
        end
        
        % match_triangle_fast 실행
        tic;
        match = match_triangle_fast(i, j, k, angle_matrix, b_vectors, ...
            catalog_fast, sensor_params.sigma, sensor_params.k_multiplier);
        exec_time = toc;
        stats.execution_times = [stats.execution_times; exec_time];
        
        % 결과 분석
        if match.success
            % Permutation 고려한 비교
            est_ids = sort([match.I, match.J, match.K]);
            gt_ids = sort(gt_cat_idx);
            
            if isequal(est_ids, gt_ids)
                stats.true_positive = stats.true_positive + 1;
            else
                stats.false_positive = stats.false_positive + 1;
                
                % 실패 케이스 저장
                fail_case.frame = frame;
                fail_case.triangle = [i, j, k];
                fail_case.gt_ids = gt_ids;
                fail_case.est_ids = est_ids;
                fail_case.unique = match.unique;
                fail_case.frequency = match.frequency;
                stats.failed_cases{end+1} = fail_case;
            end
        else
            stats.false_negative = stats.false_negative + 1;
        end
    end
end

%% 6. 결과 리포트
fprintf('\n=== 테스트 결과 ===\n');
fprintf('총 Triangle: %d개\n', stats.total_triangles);
fprintf('True Positive:  %d (%.1f%%)\n', stats.true_positive, ...
    100*stats.true_positive/stats.total_triangles);
fprintf('False Positive: %d (%.1f%%)\n', stats.false_positive, ...
    100*stats.false_positive/stats.total_triangles);
fprintf('False Negative: %d (%.1f%%)\n\n', stats.false_negative, ...
    100*stats.false_negative/stats.total_triangles);

% 성능 지표
precision = stats.true_positive / (stats.true_positive + stats.false_positive);
recall = stats.true_positive / (stats.total_triangles);
f1_score = 2 * precision * recall / (precision + recall);

fprintf('[성능 지표]\n');
fprintf('Precision: %.3f\n', precision);
fprintf('Recall:    %.3f\n', recall);
fprintf('F1-Score:  %.3f\n\n', f1_score);

% 실행시간
fprintf('[실행시간]\n');
fprintf('평균: %.3f ms\n', mean(stats.execution_times)*1000);
fprintf('중간: %.3f ms\n', median(stats.execution_times)*1000);
fprintf('최대: %.3f ms\n\n', max(stats.execution_times)*1000);

% 실패 케이스 분석
if ~isempty(stats.failed_cases)
    fprintf('[False Positive 케이스 (최대 5개)]\n');
    for i = 1:min(5, length(stats.failed_cases))
        fc = stats.failed_cases{i};
        fprintf('Frame %d, Triangle [%d,%d,%d]\n', ...
            fc.frame, fc.triangle(1), fc.triangle(2), fc.triangle(3));
        fprintf('  GT:  [%d, %d, %d]\n', fc.gt_ids);
        fprintf('  Est: [%d, %d, %d]\n', fc.est_ids);
        fprintf('  Unique: %d, Freq: %.2e\n\n', fc.unique, fc.frequency);
    end
end

% 히스토그램
figure('Position', [100, 100, 800, 300]);
histogram(stats.execution_times*1000, 30);
xlabel('실행시간 (ms)');
ylabel('빈도');
title('match\_triangle\_fast 실행시간 분포');
grid on;

fprintf('=== 테스트 완료 ===\n');