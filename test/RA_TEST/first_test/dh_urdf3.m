%% KARI ARM - DH vs URDF 랜덤 검증
clear all; close all; clc;

%% 경로 설정 및 URDF 로드
base_path = 'D:\hanul2026_scie\mppi_RA';
addpath(genpath(base_path));

urdf_files = dir(fullfile(base_path, '**', '*KARI*.urdf'));
robot = importrobot(fullfile(urdf_files(1).folder, urdf_files(1).name), 'DataFormat', 'column');

%% DH 파라미터
DH = [
    0,  -pi/2,  0.199,  0;
    0,   pi/2,  0.192,  0;
    0,  -pi/2,  0.931,  0;
    0,   pi/2, -0.192,  0;
    0,  -pi/2,  0.539,  0;
    0,   pi/2, -0.192,  0;
    0,      0,  0.185,  0;
];

%% DH FK 함수
dh_transform = @(a, alpha, d, theta) [
    cos(theta), -sin(theta)*cos(alpha),  sin(theta)*sin(alpha), a*cos(theta);
    sin(theta),  cos(theta)*cos(alpha), -cos(theta)*sin(alpha), a*sin(theta);
    0,           sin(alpha),             cos(alpha),            d;
    0,           0,                      0,                     1
];

dh_fk = @(q) compute_dh_fk(q, DH, dh_transform);

%% 랜덤 검증
num_tests = 10000;
errors = zeros(num_tests, 1);
max_err = 0;
max_err_config = [];

fprintf('=== 랜덤 %d개 구성 검증 ===\n\n', num_tests);

for i = 1:num_tests
    % 랜덤 관절각 (-pi ~ pi)
    q = -pi + 2*pi*rand(7, 1);
    
    % DH FK
    T_dh = dh_fk(q);
    p_dh = T_dh(1:3, 4);
    
    % URDF FK
    T_urdf = getTransform(robot, q, 'eef_link');
    p_urdf = T_urdf(1:3, 4);
    
    % 위치 오차 (mm)
    errors(i) = norm(p_dh - p_urdf) * 1000;
    
    if errors(i) > max_err
        max_err = errors(i);
        max_err_config = q;
    end
end

%% 결과 통계
fprintf('위치 오차 통계 [mm]:\n');
fprintf('  최소: %.6f\n', min(errors));
fprintf('  최대: %.6f\n', max(errors));
fprintf('  평균: %.6f\n', mean(errors));
fprintf('  표준편차: %.6f\n', std(errors));
fprintf('  1mm 이상 오차: %d / %d\n', sum(errors > 1), num_tests);

%% 자세(회전) 오차도 검증
fprintf('\n=== 자세(회전) 오차 검증 ===\n');
rot_errors = zeros(num_tests, 1);

for i = 1:num_tests
    q = -pi + 2*pi*rand(7, 1);
    
    T_dh = dh_fk(q);
    T_urdf = getTransform(robot, q, 'eef_link');
    
    % 회전 오차 (Frobenius norm)
    R_dh = T_dh(1:3, 1:3);
    R_urdf = T_urdf(1:3, 1:3);
    rot_errors(i) = norm(R_dh - R_urdf, 'fro');
end

fprintf('회전 오차 통계 [Frobenius]:\n');
fprintf('  최소: %.6e\n', min(rot_errors));
fprintf('  최대: %.6e\n', max(rot_errors));
fprintf('  평균: %.6e\n', mean(rot_errors));

%% 히스토그램
figure('Name', '오차 분포', 'Position', [100,100,1000,400], 'Color', 'w');

subplot(1,2,1);
histogram(errors, 50);
xlabel('위치 오차 [mm]');
ylabel('빈도');
title(sprintf('위치 오차 분포 (N=%d)', num_tests));
grid on;

subplot(1,2,2);
histogram(rot_errors, 50);
xlabel('회전 오차 [Frobenius]');
ylabel('빈도');
title('회전 오차 분포');
grid on;

%% 최대 오차 구성 상세
if max_err > 0.001
    fprintf('\n=== 최대 오차 구성 상세 ===\n');
    fprintf('관절각 [deg]: ');
    fprintf('%.1f ', rad2deg(max_err_config));
    fprintf('\n오차: %.6f mm\n', max_err);
end

fprintf('\n=== 검증 완료 ===\n');
if max(errors) < 0.01
    fprintf('결론: URDF와 DH 완벽 일치 ✓\n');
elseif max(errors) < 1
    fprintf('결론: URDF와 DH 수치오차 범위 내 일치 ✓\n');
else
    fprintf('결론: URDF와 DH 불일치 ✗\n');
end

%% DH FK 계산 함수
function T = compute_dh_fk(q, DH, dh_transform)
    T = eye(4);
    for i = 1:7
        a = DH(i,1);
        alpha = DH(i,2);
        d = DH(i,3);
        theta = q(i) + DH(i,4);
        T = T * dh_transform(a, alpha, d, theta);
    end
end