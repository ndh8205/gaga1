%% verify_kvector_catalog.m
% k-vector 카탈로그 검증 스크립트
clear; close all; clc;

fprintf('=== k-vector 카탈로그 검증 ===\n\n');

%% 1. Load catalog
if ~exist('star_catalog_fast.mat', 'file')
    error('star_catalog_fast.mat not found');
end

load('star_catalog_fast.mat');
fprintf('[로드 완료] %d stars, %d pairs\n\n', ...
    catalog_fast.N_stars, catalog_fast.N_pairs);

%% 2. 구조적 무결성 검사
fprintf('=== 구조적 무결성 ===\n');

% k_vector 단조증가
k_vec = catalog_fast.k_vector;
is_monotonic = all(diff(k_vec) >= 0);
fprintf('✓ k_vector 단조증가: %s\n', iif(is_monotonic, 'PASS', 'FAIL'));

% 경계조건
boundary_ok = (k_vec(1) == 0) && (k_vec(end) == catalog_fast.N_pairs);
fprintf('✓ 경계조건: %s (k[1]=%d, k[end]=%d)\n', ...
    iif(boundary_ok, 'PASS', 'FAIL'), k_vec(1), k_vec(end));

% sorted_angles 정렬
angles = catalog_fast.pairs_angle;  % ← 이 부분만 수정
is_sorted = issorted(angles);
fprintf('✓ pairs_angle 정렬: %s\n', iif(is_sorted, 'PASS', 'FAIL'));

if ~is_monotonic || ~boundary_ok || ~is_sorted
    error('구조적 무결성 실패!');
end

%% 3. 파라미터 검증
fprintf('\n=== 파라미터 검증 ===\n');

n = catalog_fast.N_pairs;
ymin = angles(1);
ymax = angles(end);

% 재계산
eps_machine = 2.22e-16;
delta_eps = (n - 1) * eps_machine;
m_calc = (ymax - ymin + 2*delta_eps) / (n - 1);
q_calc = ymin - m_calc - delta_eps;

% 비교
m_diff = abs(catalog_fast.m - m_calc);
q_diff = abs(catalog_fast.q - q_calc);

fprintf('m: stored=%.6e, calc=%.6e, diff=%.2e\n', ...
    catalog_fast.m, m_calc, m_diff);
fprintf('q: stored=%.6e, calc=%.6e, diff=%.2e\n', ...
    catalog_fast.q, q_calc, q_diff);

param_ok = (m_diff < 1e-12) && (q_diff < 1e-12);
fprintf('✓ 파라미터 일치: %s\n', iif(param_ok, 'PASS', 'FAIL'));

%% 4. 검색 정합성 테스트
fprintf('\n=== 검색 정합성 (100회 랜덤) ===\n');

n_tests = 100;
correct_count = 0;
time_kvector = 0;
time_linear = 0;

rng(42);  % 재현성

for test = 1:n_tests
    % 랜덤 각거리 선택
    theta_test = ymin + rand() * (ymax - ymin);
    tolerance = 0.001 + rand() * 0.005;  % 1~6 mrad
    
    % k-vector 검색
    tic;
    [I_kv, J_kv, angle_kv] = kvector_range_search_fast(theta_test, tolerance, catalog_fast);
    time_kvector = time_kvector + toc;
    
    % Linear search (ground truth)
    tic;
    ya = theta_test - tolerance;
    yb = theta_test + tolerance;
    valid = (angles >= ya) & (angles <= yb);
    I_lin = catalog_fast.pairs_I(valid);
    J_lin = catalog_fast.pairs_J(valid);
    angle_lin = angles(valid);
    time_linear = time_linear + toc;
    
    % 비교
    if isequal(I_kv, I_lin) && isequal(J_kv, J_lin) && isequal(angle_kv, angle_lin)
        correct_count = correct_count + 1;
    else
        fprintf('  Test %d FAIL: theta=%.4f, tol=%.4f\n', test, theta_test, tolerance);
        fprintf('    k-vector: %d results\n', length(I_kv));
        fprintf('    linear:   %d results\n', length(I_lin));
    end
end

accuracy = 100 * correct_count / n_tests;
avg_time_kv = time_kvector / n_tests * 1000;  % ms
avg_time_lin = time_linear / n_tests * 1000;
speedup = time_linear / time_kvector;

fprintf('✓ 정확도: %.1f%% (%d/%d)\n', accuracy, correct_count, n_tests);
fprintf('✓ k-vector 평균 시간: %.3f ms\n', avg_time_kv);
fprintf('✓ linear 평균 시간: %.3f ms\n', avg_time_lin);
fprintf('✓ 속도 향상: %.1fx\n', speedup);

%% 5. Coverage 확인
fprintf('\n=== Coverage 확인 ===\n');

N_stars = catalog_fast.N_stars;
max_possible_pairs = N_stars * (N_stars - 1) / 2;

fprintf('✓ 전체 별: %d개\n', N_stars);
fprintf('✓ 최대 가능 쌍: %d개\n', max_possible_pairs);
fprintf('✓ FOV 내 쌍: %d개 (%.1f%%)\n', ...
    catalog_fast.N_pairs, 100*catalog_fast.N_pairs/max_possible_pairs);
fprintf('✓ 각거리 범위: [%.4f, %.4f] rad ([%.2f, %.2f]°)\n', ...
    ymin, ymax, rad2deg(ymin), rad2deg(ymax));
fprintf('✓ FOV 설정: %.2f° (%.4f rad)\n', ...
    rad2deg(catalog_fast.FOV_rad), catalog_fast.FOV_rad);

% 각거리 분포 히스토그램
figure('Position', [100, 100, 800, 400]);

subplot(1,2,1);
histogram(angles, 50, 'Normalization', 'probability');
xlabel('각거리 (rad)');
ylabel('확률');
title('각거리 분포');
grid on;

subplot(1,2,2);
histogram(rad2deg(angles), 50, 'Normalization', 'probability');
xlabel('각거리 (deg)');
ylabel('확률');
title('각거리 분포');
grid on;

%% 6. 최종 리포트
fprintf('\n=== 검증 요약 ===\n');
all_pass = is_monotonic && boundary_ok && is_sorted && param_ok && (accuracy == 100);

if all_pass
    fprintf('✓✓✓ 모든 테스트 통과! k-vector 정상 동작 확인 ✓✓✓\n');
else
    fprintf('✗✗✗ 일부 테스트 실패. 카탈로그 재생성 필요 ✗✗✗\n');
end

fprintf('\n');

%% Helper function
function result = iif(condition, true_val, false_val)
    if condition
        result = true_val;
    else
        result = false_val;
    end
end