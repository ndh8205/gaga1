%% debug_03_kvector_search.m
% k-vector 범위 검색 정확성 및 성능 검증
clear; close all; clc;

fprintf('=== k-vector 검색 검증 ===\n\n');

%% 1. 카탈로그 로드
if ~exist('star_catalog_kvector.mat', 'file')
    error('먼저 debug_01_kvector_catalog.m을 실행하세요.');
end
load('star_catalog_kvector.mat');
fprintf('카탈로그 로드 완료\n');
fprintf('  별 개수: %d\n', catalog_data.N_stars);
fprintf('  별쌍 개수: %d\n\n', catalog_data.N_pairs);

%% 2. Binary Search와 비교 (정확성 검증)
fprintf('=== Binary Search vs k-vector 정확성 비교 ===\n');

sigma = 50e-6;
k_multiplier = 6.4;
tolerance = k_multiplier * sigma;

% 테스트 각도들
test_angles = linspace(min(catalog_data.sorted_angles), ...
                       max(catalog_data.sorted_angles), 20);

mismatch_count = 0;

for i = 1:length(test_angles)
    angle = test_angles(i);
    
    % Binary Search (ground truth)
    ya = angle - tolerance;
    yb = angle + tolerance;
    bs_idx = find(catalog_data.sorted_angles >= ya & ...
                  catalog_data.sorted_angles <= yb);
    bs_count = length(bs_idx);
    
    % k-vector Search
    [kv_candidates, ~, ~] = kvector_range_search(angle, tolerance, catalog_data);
    kv_count = length(kv_candidates);
    
    % 비교
    match = (bs_count == kv_count);
    if ~match
        mismatch_count = mismatch_count + 1;
    end
    
    fprintf('각도 %.4f deg: BS=%3d, KV=%3d %s\n', ...
        rad2deg(angle), bs_count, kv_count, ...
        char(10004*match + 10008*(~match)));
end

if mismatch_count == 0
    fprintf('\n✓ 모든 테스트 일치!\n');
else
    fprintf('\n✗ %d개 불일치 감지\n', mismatch_count);
end

%% 3. 성능 비교 (Binary Search vs k-vector)
fprintf('\n=== 성능 비교 (1000회 검색) ===\n');

N_tests = 1000;
test_angles_perf = rand(N_tests, 1) * ...
    (max(catalog_data.sorted_angles) - min(catalog_data.sorted_angles)) + ...
    min(catalog_data.sorted_angles);

% Binary Search 타이밍
tic;
for i = 1:N_tests
    angle = test_angles_perf(i);
    ya = angle - tolerance;
    yb = angle - tolerance;
    idx = find(catalog_data.sorted_angles >= ya & ...
               catalog_data.sorted_angles <= yb);
end
bs_time = toc;

% k-vector 타이밍
tic;
for i = 1:N_tests
    angle = test_angles_perf(i);
    [candidates, ~, ~] = kvector_range_search(angle, tolerance, catalog_data);
end
kv_time = toc;

fprintf('Binary Search: %.4f ms/검색\n', bs_time/N_tests*1000);
fprintf('k-vector:      %.4f ms/검색\n', kv_time/N_tests*1000);
fprintf('속도 향상:     %.1f배\n', bs_time/kv_time);

%% 4. Extraneous Elements 통계
fprintf('\n=== Extraneous Elements 통계 ===\n');

E0_expected = catalog_data.N_pairs / (catalog_data.N_pairs - 1);
fprintf('이론적 E0: %.4f\n', E0_expected);

extraneous_counts = zeros(N_tests, 1);

for i = 1:N_tests
    angle = test_angles_perf(i);
    
    % k-vector 검색 (경계 정리 전)
    ya = angle - tolerance;
    yb = angle + tolerance;
    j_b = floor((ya - catalog_data.q) / catalog_data.m);
    j_t = ceil((yb - catalog_data.q) / catalog_data.m);
    j_b = max(1, min(j_b, catalog_data.N_pairs));
    j_t = max(1, min(j_t, catalog_data.N_pairs));
    k_start = catalog_data.k_vector(j_b) + 1;
    k_end = catalog_data.k_vector(j_t);
    
    if k_end >= k_start
        candidates_raw = catalog_data.sorted_pairs(k_start:k_end);
        
        % 실제 범위 내 개수
        in_range = sum([candidates_raw.angle] >= ya & [candidates_raw.angle] <= yb);
        extraneous_counts(i) = length(candidates_raw) - in_range;
    end
end

fprintf('실험적 E0: %.4f (std: %.4f)\n', ...
    mean(extraneous_counts), std(extraneous_counts));

figure('Name', 'Extraneous Elements', 'Position', [100, 100, 800, 400]);
histogram(extraneous_counts, 'BinMethod', 'integers', 'FaceColor', [0.2 0.6 0.8]);
grid on;
xlabel('Number of Extraneous Elements');
ylabel('Frequency');
title(sprintf('E_0 Distribution (mean: %.2f)', mean(extraneous_counts)));

%% 5. 후보 개수 vs 각도
fprintf('\n=== 후보 개수 분포 ===\n');

angles_sample = linspace(min(catalog_data.sorted_angles), ...
                        max(catalog_data.sorted_angles), 100);
candidate_counts = zeros(size(angles_sample));

for i = 1:length(angles_sample)
    [candidates, ~, ~] = kvector_range_search(angles_sample(i), tolerance, catalog_data);
    candidate_counts(i) = length(candidates);
end

fprintf('평균 후보 개수: %.1f\n', mean(candidate_counts));
fprintf('최대 후보 개수: %d\n', max(candidate_counts));

figure('Name', 'Candidates vs Angle', 'Position', [100, 100, 1000, 400]);
plot(rad2deg(angles_sample), candidate_counts, 'b-', 'LineWidth', 1.5);
grid on;
xlabel('Search Angle (deg)');
ylabel('Number of Candidates');
title('k-vector Search Candidates');

fprintf('\n=== 테스트 완료 ===\n');