%% 두 URDF 모델 비교 및 검증 스크립트
clear all; close all; clc;

%% 0. 경로 설정 - 하위 폴더 전체 탐색
base_path = 'D:\hanul2026_scie\mppi_RA';
addpath(genpath(base_path));
fprintf('경로 추가 완료: %s\n', base_path);

%% 1. URDF 파일 자동 탐색
fprintf('\n=== URDF 파일 탐색 ===\n');
urdf_files = dir(fullfile(base_path, '**', '*.urdf'));
fprintf('발견된 URDF 파일:\n');
for i = 1:length(urdf_files)
    fprintf('  [%d] %s\n', i, fullfile(urdf_files(i).folder, urdf_files(i).name));
end

% URDF 파일 자동 매칭
kari_path = ''; m1013_path = '';
for i = 1:length(urdf_files)
    fname = lower(urdf_files(i).name);
    fpath = fullfile(urdf_files(i).folder, urdf_files(i).name);
    if contains(fname, 'kari') || contains(fname, 'asm')
        kari_path = fpath;
    elseif contains(fname, 'm1013')
        m1013_path = fpath;
    end
end

fprintf('\n=== URDF 모델 로드 ===\n');

% ASM_KARI_ARM (7DOF)
robot_kari = importrobot(kari_path, 'DataFormat', 'column');
robot_kari.Gravity = [0 0 -9.81];
fprintf('KARI ARM 로드 완료: %s (%d DOF)\n', kari_path, numel(homeConfiguration(robot_kari)));

% m1013 (6DOF)
robot_m1013 = importrobot(m1013_path, 'DataFormat', 'column');
robot_m1013.Gravity = [0 0 -9.81];
fprintf('m1013 로드 완료: %s (%d DOF)\n', m1013_path, numel(homeConfiguration(robot_m1013)));

%% 2. 기본 정보 비교
fprintf('\n=== 기본 정보 비교 ===\n');
fprintf('%-20s | %-15s | %-15s\n', '항목', 'KARI_ARM', 'm1013');
fprintf('------------------------------------------------------------\n');

% DOF
dof_kari = numel(homeConfiguration(robot_kari));
dof_m1013 = numel(homeConfiguration(robot_m1013));
fprintf('%-20s | %-15d | %-15d\n', 'DOF', dof_kari, dof_m1013);

% 총 질량
mass_kari = 0; mass_m1013 = 0;
for i = 1:length(robot_kari.Bodies)
    mass_kari = mass_kari + robot_kari.Bodies{i}.Mass;
end
for i = 1:length(robot_m1013.Bodies)
    mass_m1013 = mass_m1013 + robot_m1013.Bodies{i}.Mass;
end
fprintf('%-20s | %-15.3f | %-15.3f\n', '총 질량 [kg]', mass_kari, mass_m1013);

% 링크 수
fprintf('%-20s | %-15d | %-15d\n', '링크 수', length(robot_kari.Bodies), length(robot_m1013.Bodies));

%% 3. 링크별 질량 및 CoM 출력
fprintf('\n=== KARI ARM 링크 정보 ===\n');
fprintf('%-12s | %8s | %-30s\n', '링크', '질량[kg]', 'CoM [x, y, z]');
fprintf('------------------------------------------------------------\n');
for i = 1:length(robot_kari.Bodies)
    body = robot_kari.Bodies{i};
    com = body.CenterOfMass;
    fprintf('%-12s | %8.3f | [%8.4f, %8.4f, %8.4f]\n', body.Name, body.Mass, com(1), com(2), com(3));
end

fprintf('\n=== m1013 링크 정보 ===\n');
fprintf('%-12s | %8s | %-30s\n', '링크', '질량[kg]', 'CoM [x, y, z]');
fprintf('------------------------------------------------------------\n');
for i = 1:length(robot_m1013.Bodies)
    body = robot_m1013.Bodies{i};
    com = body.CenterOfMass;
    fprintf('%-12s | %8.3f | [%8.4f, %8.4f, %8.4f]\n', body.Name, body.Mass, com(1), com(2), com(3));
end

%% 4. 관절 축 확인
fprintf('\n=== KARI ARM 관절 축 ===\n');
for i = 1:length(robot_kari.Bodies)
    jnt = robot_kari.Bodies{i}.Joint;
    if ~strcmp(jnt.Type, 'fixed')
        fprintf('%-12s: 축 = [%g, %g, %g]\n', jnt.Name, jnt.JointAxis);
    end
end

fprintf('\n=== m1013 관절 축 ===\n');
for i = 1:length(robot_m1013.Bodies)
    jnt = robot_m1013.Bodies{i}.Joint;
    if ~strcmp(jnt.Type, 'fixed')
        fprintf('%-12s: 축 = [%g, %g, %g]\n', jnt.Name, jnt.JointAxis);
    end
end

%% 5. 시각화 - 영점 구성
figure('Name', '영점 구성 비교', 'Position', [100, 100, 1400, 600]);

% KARI ARM
subplot(1,2,1);
q_kari = zeros(dof_kari, 1);
show(robot_kari, q_kari, 'Visuals', 'off', 'Frames', 'on');
title('KARI ARM (7DOF) - 영점');
view(45, 20); grid on; axis equal;
xlabel('X [m]'); ylabel('Y [m]'); zlabel('Z [m]');

% m1013
subplot(1,2,2);
q_m1013 = zeros(dof_m1013, 1);
show(robot_m1013, q_m1013, 'Visuals', 'off', 'Frames', 'on');
title('m1013 (6DOF) - 영점');
view(45, 20); grid on; axis equal;
xlabel('X [m]'); ylabel('Y [m]'); zlabel('Z [m]');

%% 6. EE 위치 확인 (여러 구성)
fprintf('\n=== 엔드이펙터 위치 비교 ===\n');

% KARI EE 링크
ee_kari = 'eef_link';
ee_m1013 = 'link_6';

% 테스트 구성
test_angles_deg = [0, 30, 45, 90];

fprintf('\n[KARI ARM] EE: %s\n', ee_kari);
fprintf('구성 (J1만 회전) | EE 위치 [x, y, z]\n');
fprintf('----------------------------------------\n');
for ang = test_angles_deg
    q = zeros(dof_kari, 1);
    q(1) = deg2rad(ang);
    T = getTransform(robot_kari, q, ee_kari);
    p = T(1:3, 4);
    fprintf('%3d deg          | [%7.4f, %7.4f, %7.4f]\n', ang, p);
end

fprintf('\n[m1013] EE: %s\n', ee_m1013);
fprintf('구성 (J1만 회전) | EE 위치 [x, y, z]\n');
fprintf('----------------------------------------\n');
for ang = test_angles_deg
    q = zeros(dof_m1013, 1);
    q(1) = deg2rad(ang);
    T = getTransform(robot_m1013, q, ee_m1013);
    p = T(1:3, 4);
    fprintf('%3d deg          | [%7.4f, %7.4f, %7.4f]\n', ang, p);
end

%% 7. 영점에서 EE 도달거리
fprintf('\n=== 영점 EE 도달거리 ===\n');
T_kari = getTransform(robot_kari, zeros(dof_kari,1), ee_kari);
T_m1013 = getTransform(robot_m1013, zeros(dof_m1013,1), ee_m1013);

p_kari = T_kari(1:3,4);
p_m1013 = T_m1013(1:3,4);

fprintf('KARI ARM EE: [%7.4f, %7.4f, %7.4f], 거리: %.4f m\n', p_kari, norm(p_kari));
fprintf('m1013 EE:    [%7.4f, %7.4f, %7.4f], 거리: %.4f m\n', p_m1013, norm(p_m1013));

%% 8. 작업공간 샘플링 비교
fprintf('\n=== 작업공간 샘플링 ===\n');
num_samples = 500;

figure('Name', '작업공간 비교', 'Position', [100, 100, 1400, 600]);

% KARI ARM 작업공간
subplot(1,2,1);
ws_kari = zeros(3, num_samples);
valid_kari = 0;
for i = 1:num_samples
    q = -pi + 2*pi*rand(dof_kari, 1);
    try
        T = getTransform(robot_kari, q, ee_kari);
        p = T(1:3, 4);
        if all(~isnan(p)) && norm(p) < 5
            valid_kari = valid_kari + 1;
            ws_kari(:, valid_kari) = p;
        end
    catch
    end
end
ws_kari = ws_kari(:, 1:valid_kari);
scatter3(ws_kari(1,:), ws_kari(2,:), ws_kari(3,:), 5, 'b', 'filled');
title(sprintf('KARI ARM 작업공간 (%d점)', valid_kari));
view(45, 20); grid on; axis equal;
xlabel('X [m]'); ylabel('Y [m]'); zlabel('Z [m]');

% m1013 작업공간
subplot(1,2,2);
ws_m1013 = zeros(3, num_samples);
valid_m1013 = 0;
for i = 1:num_samples
    q = -pi + 2*pi*rand(dof_m1013, 1);
    try
        T = getTransform(robot_m1013, q, ee_m1013);
        p = T(1:3, 4);
        if all(~isnan(p)) && norm(p) < 5
            valid_m1013 = valid_m1013 + 1;
            ws_m1013(:, valid_m1013) = p;
        end
    catch
    end
end
ws_m1013 = ws_m1013(:, 1:valid_m1013);
scatter3(ws_m1013(1,:), ws_m1013(2,:), ws_m1013(3,:), 5, 'r', 'filled');
title(sprintf('m1013 작업공간 (%d점)', valid_m1013));
view(45, 20); grid on; axis equal;
xlabel('X [m]'); ylabel('Y [m]'); zlabel('Z [m]');

fprintf('KARI 작업공간 범위: X[%.2f, %.2f], Y[%.2f, %.2f], Z[%.2f, %.2f]\n', ...
    min(ws_kari(1,:)), max(ws_kari(1,:)), ...
    min(ws_kari(2,:)), max(ws_kari(2,:)), ...
    min(ws_kari(3,:)), max(ws_kari(3,:)));
fprintf('m1013 작업공간 범위: X[%.2f, %.2f], Y[%.2f, %.2f], Z[%.2f, %.2f]\n', ...
    min(ws_m1013(1,:)), max(ws_m1013(1,:)), ...
    min(ws_m1013(2,:)), max(ws_m1013(2,:)), ...
    min(ws_m1013(3,:)), max(ws_m1013(3,:)));

%% 9. 동역학 검증 - 질량행렬 조건수
fprintf('\n=== 동역학 검증 (영점) ===\n');

M_kari = massMatrix(robot_kari, zeros(dof_kari, 1));
M_m1013 = massMatrix(robot_m1013, zeros(dof_m1013, 1));

fprintf('KARI ARM 질량행렬 조건수: %.2f\n', cond(M_kari));
fprintf('m1013 질량행렬 조건수: %.2f\n', cond(M_m1013));

fprintf('\nKARI ARM 질량행렬 대각성분:\n');
disp(diag(M_kari)');
fprintf('m1013 질량행렬 대각성분:\n');
disp(diag(M_m1013)');

%% 10. CoM 위치 시각화 (m1013 글로벌 좌표 검증)
fprintf('\n=== CoM 글로벌 좌표 검증 (m1013) ===\n');
fprintf('주의: URDF의 CoM이 글로벌 좌표로 보임\n');

figure('Name', 'CoM 위치 검증', 'Position', [100, 100, 800, 600]);
q = zeros(dof_m1013, 1);
show(robot_m1013, q, 'Visuals', 'off', 'Frames', 'on');
hold on;

% 각 링크의 CoM 표시 (URDF에 기록된 값 그대로)
com_z_values = [0.050723, 0.143327, 0.418558, 0.781807, 1.148, 1.33103, 1.40022];
link_names = {'base_link', 'link_1', 'link_2', 'link_3', 'link_4', 'link_5', 'link_6'};

for i = 1:length(link_names)
    plot3(0, 0, com_z_values(i), 'ro', 'MarkerSize', 10, 'LineWidth', 2);
    text(0.05, 0, com_z_values(i), link_names{i}, 'FontSize', 8);
end
title('m1013 CoM Z값 (URDF 기록값) - 글로벌 좌표 의심');
view(0, 0); grid on;
xlabel('X [m]'); ylabel('Y [m]'); zlabel('Z [m]');

fprintf('m1013 CoM Z값들이 베이스부터 누적되는 패턴 확인\n');
fprintf('이는 URDF 작성 오류일 가능성 높음 (로컬 좌표여야 함)\n');

%% 11. 테스트 구성 시각화
figure('Name', '테스트 구성 비교', 'Position', [100, 100, 1400, 800]);

% 테스트 구성들
test_configs_kari = {
    '영점', zeros(dof_kari, 1);
    '팔 굽힘', [0; pi/4; 0; pi/4; 0; pi/4; 0];
    '전체 45도', ones(dof_kari,1)*pi/4
};

test_configs_m1013 = {
    '영점', zeros(dof_m1013, 1);
    '팔 굽힘', [0; pi/4; 0; pi/4; 0; 0];
    '전체 45도', ones(dof_m1013,1)*pi/4
};

for i = 1:3
    % KARI
    subplot(2,3,i);
    show(robot_kari, test_configs_kari{i,2}, 'Visuals', 'off', 'Frames', 'on');
    title(sprintf('KARI: %s', test_configs_kari{i,1}));
    view(45, 20); grid on; axis equal;
    
    % m1013
    subplot(2,3,i+3);
    show(robot_m1013, test_configs_m1013{i,2}, 'Visuals', 'off', 'Frames', 'on');
    title(sprintf('m1013: %s', test_configs_m1013{i,1}));
    view(45, 20); grid on; axis equal;
end

fprintf('\n=== 비교 완료 ===\n');