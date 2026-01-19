%% KARI ARM - URDF vs DH 파라미터 비교
clear all; close all; clc;

%% 경로 설정 및 로드
base_path = 'D:\hanul2026_scie\mppi_RA';
addpath(genpath(base_path));

urdf_files = dir(fullfile(base_path, '**', '*KARI*.urdf'));
robot = importrobot(fullfile(urdf_files(1).folder, urdf_files(1).name), 'DataFormat', 'column');
robot.Gravity = [0 0 -9.81];
dof = numel(homeConfiguration(robot));

%% 레퍼런스 DH 파라미터 (mm)
% d값: 관절 축 방향(Z) 거리
dh_d = [199, 192, 931, -192, 539, -192, 185]; % mm

fprintf('=== 레퍼런스 DH d값 [mm] ===\n');
for i = 1:7
    fprintf('d%d = %d\n', i, dh_d(i));
end
fprintf('총합: %d mm\n\n', sum(dh_d));

%% URDF 영점에서 각 프레임 글로벌 위치 계산
q = zeros(dof, 1);

link_names = {'base', 'link1', 'link2', 'link3', 'link4', 'link5', 'link6', 'link7', 'eef_link'};

fprintf('=== URDF 프레임 글로벌 위치 (영점) ===\n');
fprintf('%-10s | %10s | %10s | %10s\n', '프레임', 'X[mm]', 'Y[mm]', 'Z[mm]');
fprintf('----------------------------------------------\n');

positions = zeros(length(link_names), 3);
for i = 1:length(link_names)
    try
        T = getTransform(robot, q, link_names{i});
        positions(i,:) = T(1:3,4)' * 1000; % mm로 변환
        fprintf('%-10s | %10.1f | %10.1f | %10.1f\n', link_names{i}, positions(i,:));
    catch
        fprintf('%-10s | 변환 실패\n', link_names{i});
    end
end

%% 연속 프레임 간 거리 계산
fprintf('\n=== 프레임 간 거리 vs DH d값 ===\n');
fprintf('%-15s | %8s | %8s | %8s | %8s | %8s\n', '구간', 'dX', 'dY', 'dZ', '|거리|', 'DH_d');
fprintf('------------------------------------------------------------------------\n');

for i = 1:length(link_names)-1
    dx = positions(i+1,1) - positions(i,1);
    dy = positions(i+1,2) - positions(i,2);
    dz = positions(i+1,3) - positions(i,3);
    dist = norm(positions(i+1,:) - positions(i,:));
    
    if i <= 7
        dh_val = dh_d(i);
    else
        dh_val = 0;
    end
    
    fprintf('%-15s | %8.1f | %8.1f | %8.1f | %8.1f | %8d\n', ...
        sprintf('%s->%s', link_names{i}, link_names{i+1}), dx, dy, dz, dist, dh_val);
end

%% 누적 Z vs DH 누적
fprintf('\n=== 누적 높이 비교 ===\n');
fprintf('URDF EEF Z: %.1f mm\n', positions(end,3));
fprintf('DH d합계:   %.1f mm\n', sum(dh_d));

%% 시각화
figure('Name', 'URDF vs DH 비교', 'Position', [100,100,1200,600], 'Color', 'w');

subplot(1,2,1);
show(robot, q, 'Visuals', 'off', 'Frames', 'on');
hold on;
plot3(positions(:,1)/1000, positions(:,2)/1000, positions(:,3)/1000, ...
    'ro-', 'LineWidth', 2, 'MarkerSize', 10, 'MarkerFaceColor', 'r');
title('URDF 프레임 위치');
view(45, 25); grid on; axis equal;
xlabel('X[m]'); ylabel('Y[m]'); zlabel('Z[m]');

subplot(1,2,2);
% DH 기반 예상 위치 (Z축 기준 단순화)
dh_cumsum = [0, cumsum(dh_d)];
bar(dh_cumsum, 'FaceColor', [0.3 0.6 0.9]);
hold on;
plot(1:9, positions(:,3), 'ro-', 'LineWidth', 2, 'MarkerSize', 8);
legend('DH 누적 Z', 'URDF 실제 Z');
title('Z축 높이 비교');
xlabel('프레임 인덱스'); ylabel('Z [mm]');
grid on;

fprintf('\n=== 분석 완료 ===\n');