%% DH vs 원본 URDF vs CAD URDF 비교
clear all; close all; clc;

%% 경로 설정
base_path = 'D:\hanul2026_scie\mppi_RA';
addpath(genpath(base_path));

%% 1. 모든 KARI URDF 탐색
all_urdf = dir(fullfile(base_path, '**', '*KARI*.urdf'));
fprintf('발견된 URDF:\n');
for i = 1:length(all_urdf)
    fprintf('  [%d] %s\n', i, fullfile(all_urdf(i).folder, all_urdf(i).name));
end

%% 2. 원본 URDF 로드 (CAD 아닌것)
orig_urdf_path = '';
for i = 1:length(all_urdf)
    if ~contains(all_urdf(i).name, 'CAD', 'IgnoreCase', true)
        orig_urdf_path = fullfile(all_urdf(i).folder, all_urdf(i).name);
        break;
    end
end
robot_orig = importrobot(orig_urdf_path, 'DataFormat', 'column');
fprintf('\n원본 URDF 로드: %s\n', orig_urdf_path);

%% 3. CAD URDF 로드 (CAD 포함된것)
cad_urdf_path = '';
for i = 1:length(all_urdf)
    if contains(all_urdf(i).name, 'CAD', 'IgnoreCase', true)
        cad_urdf_path = fullfile(all_urdf(i).folder, all_urdf(i).name);
        break;
    end
end
if isempty(cad_urdf_path)
    error('CAD URDF 파일을 찾을 수 없음. ASM_KARI_ARM_CAD.urdf 파일을 아무 하위폴더에 넣으세요.');
end
robot_cad = importrobot(cad_urdf_path, 'DataFormat', 'column');
fprintf('CAD URDF 로드: %s\n', cad_urdf_path);

%% 3. DH 파라미터
DH = [
    0,  -pi/2,  0.199,  0;
    0,   pi/2,  0.192,  0;
    0,  -pi/2,  0.931,  0;
    0,   pi/2, -0.192,  0;
    0,  -pi/2,  0.539,  0;
    0,   pi/2, -0.192,  0;
    0,      0,  0.185,  0;
];

dh_transform = @(a, alpha, d, theta) [
    cos(theta), -sin(theta)*cos(alpha),  sin(theta)*sin(alpha), a*cos(theta);
    sin(theta),  cos(theta)*cos(alpha), -cos(theta)*sin(alpha), a*sin(theta);
    0,           sin(alpha),             cos(alpha),            d;
    0,           0,                      0,                     1
];

%% 랜덤 검증 (1000개)
num_tests = 1000;
err_orig = zeros(num_tests, 1);
err_cad = zeros(num_tests, 1);
err_orig_cad = zeros(num_tests, 1);

fprintf('\n=== 랜덤 %d개 구성 검증 ===\n', num_tests);

for i = 1:num_tests
    q = -pi + 2*pi*rand(7, 1);
    
    % DH FK
    T_dh = eye(4);
    for j = 1:7
        T_dh = T_dh * dh_transform(DH(j,1), DH(j,2), DH(j,3), q(j)+DH(j,4));
    end
    p_dh = T_dh(1:3, 4);
    
    % 원본 URDF FK
    T_orig = getTransform(robot_orig, q, 'eef_link');
    p_orig = T_orig(1:3, 4);
    
    % CAD URDF FK
    T_cad = getTransform(robot_cad, q, 'eef_link');
    p_cad = T_cad(1:3, 4);
    
    % 오차 (mm)
    err_orig(i) = norm(p_dh - p_orig) * 1000;
    err_cad(i) = norm(p_dh - p_cad) * 1000;
    err_orig_cad(i) = norm(p_orig - p_cad) * 1000;
end

%% 결과 출력
fprintf('\n=== EE 위치 오차 [mm] ===\n');
fprintf('%-20s | %10s | %10s | %10s | %10s\n', '비교', '최소', '최대', '평균', '표준편차');
fprintf('------------------------------------------------------------------------\n');
fprintf('%-20s | %10.4f | %10.4f | %10.4f | %10.4f\n', 'DH vs 원본URDF', min(err_orig), max(err_orig), mean(err_orig), std(err_orig));
fprintf('%-20s | %10.4f | %10.4f | %10.4f | %10.4f\n', 'DH vs CAD_URDF', min(err_cad), max(err_cad), mean(err_cad), std(err_cad));
fprintf('%-20s | %10.4f | %10.4f | %10.4f | %10.4f\n', '원본URDF vs CAD_URDF', min(err_orig_cad), max(err_orig_cad), mean(err_orig_cad), std(err_orig_cad));

%% 영점에서 상세 비교
fprintf('\n=== 영점 구성 상세 비교 ===\n');
q = zeros(7, 1);

% DH
T_dh = eye(4);
for j = 1:7
    T_dh = T_dh * dh_transform(DH(j,1), DH(j,2), DH(j,3), q(j)+DH(j,4));
end

T_orig = getTransform(robot_orig, q, 'eef_link');
T_cad = getTransform(robot_cad, q, 'eef_link');

fprintf('DH EE:        [%8.3f, %8.3f, %8.3f] mm\n', T_dh(1:3,4)*1000);
fprintf('원본 URDF EE: [%8.3f, %8.3f, %8.3f] mm\n', T_orig(1:3,4)*1000);
fprintf('CAD URDF EE:  [%8.3f, %8.3f, %8.3f] mm\n', T_cad(1:3,4)*1000);

%% 각 링크 위치 비교 (영점)
fprintf('\n=== 영점에서 각 링크 위치 [mm] ===\n');
fprintf('%-8s | %-28s | %-28s | %-28s\n', '링크', 'DH', '원본URDF', 'CAD_URDF');
fprintf('----------------------------------------------------------------------------------------------------\n');

link_names = {'link1', 'link2', 'link3', 'link4', 'link5', 'link6', 'link7', 'eef_link'};

T_dh = eye(4);
for i = 1:7
    T_dh = T_dh * dh_transform(DH(i,1), DH(i,2), DH(i,3), DH(i,4));
    p_dh = T_dh(1:3,4) * 1000;
    
    T_orig = getTransform(robot_orig, q, link_names{i});
    p_orig = T_orig(1:3,4) * 1000;
    
    T_cad = getTransform(robot_cad, q, link_names{i});
    p_cad = T_cad(1:3,4) * 1000;
    
    fprintf('%-8s | [%7.1f, %7.1f, %7.1f] | [%7.1f, %7.1f, %7.1f] | [%7.1f, %7.1f, %7.1f]\n', ...
        link_names{i}, p_dh, p_orig, p_cad);
end

% EEF
T_orig = getTransform(robot_orig, q, 'eef_link');
T_cad = getTransform(robot_cad, q, 'eef_link');
fprintf('%-8s | [%7.1f, %7.1f, %7.1f] | [%7.1f, %7.1f, %7.1f] | [%7.1f, %7.1f, %7.1f]\n', ...
    'eef_link', T_dh(1:3,4)*1000, T_orig(1:3,4)*1000, T_cad(1:3,4)*1000);

%% 시각화
figure('Name', '3개 모델 비교', 'Position', [50, 50, 1600, 500], 'Color', 'w');

q_test = zeros(7, 1);

% 원본 URDF
subplot(1,3,1);
show(robot_orig, q_test, 'Visuals', 'off', 'Frames', 'on');
title('원본 URDF (Y 오프셋)');
view(45, 25); grid on; axis equal;
xlabel('X'); ylabel('Y'); zlabel('Z');

% CAD URDF
subplot(1,3,2);
show(robot_cad, q_test, 'Visuals', 'off', 'Frames', 'on');
title('CAD URDF (X 오프셋)');
view(45, 25); grid on; axis equal;
xlabel('X'); ylabel('Y'); zlabel('Z');

% 오버레이 비교
subplot(1,3,3);
hold on;

% 원본 URDF 링크 위치
pos_orig = zeros(9, 3);
all_links = {'base', 'link1', 'link2', 'link3', 'link4', 'link5', 'link6', 'link7', 'eef_link'};
for i = 1:9
    T = getTransform(robot_orig, q_test, all_links{i});
    pos_orig(i,:) = T(1:3,4)';
end
plot3(pos_orig(:,1), pos_orig(:,2), pos_orig(:,3), 'b-o', 'LineWidth', 2, 'MarkerSize', 8, 'DisplayName', '원본 URDF');

% CAD URDF 링크 위치
pos_cad = zeros(9, 3);
for i = 1:9
    T = getTransform(robot_cad, q_test, all_links{i});
    pos_cad(i,:) = T(1:3,4)';
end
plot3(pos_cad(:,1), pos_cad(:,2), pos_cad(:,3), 'r-s', 'LineWidth', 2, 'MarkerSize', 8, 'DisplayName', 'CAD URDF');

% DH 위치
pos_dh = zeros(8, 3);
T = eye(4);
pos_dh(1,:) = [0,0,0];
for i = 1:7
    T = T * dh_transform(DH(i,1), DH(i,2), DH(i,3), DH(i,4));
    pos_dh(i+1,:) = T(1:3,4)';
end
plot3(pos_dh(:,1), pos_dh(:,2), pos_dh(:,3), 'g-^', 'LineWidth', 2, 'MarkerSize', 8, 'DisplayName', 'DH');

legend('Location', 'best');
title('3개 모델 오버레이');
view(45, 25); grid on; axis equal;
xlabel('X[m]'); ylabel('Y[m]'); zlabel('Z[m]');

%% 관절 축 비교
fprintf('\n=== 관절 축 비교 ===\n');
fprintf('%-8s | %-15s | %-15s\n', '관절', '원본URDF', 'CAD_URDF');
fprintf('--------------------------------------------\n');
for i = 1:length(robot_orig.Bodies)
    jnt_orig = robot_orig.Bodies{i}.Joint;
    if ~strcmp(jnt_orig.Type, 'fixed')
        jnt_cad = robot_cad.Bodies{i}.Joint;
        fprintf('%-8s | [%g, %g, %g] | [%g, %g, %g]\n', ...
            jnt_orig.Name, jnt_orig.JointAxis, jnt_cad.JointAxis);
    end
end

fprintf('\n=== 비교 완료 ===\n');