%% KARI ARM - DH FK vs URDF FK 비교
clear all; close all; clc;

%% 경로 설정 및 URDF 로드
base_path = 'D:\hanul2026_scie\mppi_RA';
addpath(genpath(base_path));

urdf_files = dir(fullfile(base_path, '**', '*KARI*.urdf'));
robot = importrobot(fullfile(urdf_files(1).folder, urdf_files(1).name), 'DataFormat', 'column');
dof = numel(homeConfiguration(robot));

%% 레퍼런스 DH 파라미터 (표 기준)
% 좌표계 {i-1} | 조인트 Ji | 링크번호 Li | 링크길이 ai | 관절 오프셋 di | 링크 비틀림 αi | 초기관절각도 βi
% {0}          | 1         | 1           | 0           | d1(199)       | -90            | 0
% {1}          | 2         | 2           | 0           | d2(192)       | 90             | 0
% ...

% DH 파라미터 [a, alpha, d, theta_offset] (mm -> m, deg -> rad)
DH = [
    0,  -pi/2,  0.199,  0;    % Joint 1
    0,   pi/2,  0.192,  0;    % Joint 2
    0,  -pi/2,  0.931,  0;    % Joint 3
    0,   pi/2, -0.192,  0;    % Joint 4
    0,  -pi/2,  0.539,  0;    % Joint 5
    0,   pi/2, -0.192,  0;    % Joint 6
    0,      0,  0.185,  0;    % Joint 7
];

%% DH Forward Kinematics 함수
dh_transform = @(a, alpha, d, theta) [
    cos(theta), -sin(theta)*cos(alpha),  sin(theta)*sin(alpha), a*cos(theta);
    sin(theta),  cos(theta)*cos(alpha), -cos(theta)*sin(alpha), a*sin(theta);
    0,           sin(alpha),             cos(alpha),            d;
    0,           0,                      0,                     1
];

%% 테스트 구성들
test_configs = {
    '영점',        zeros(7,1);
    'J1=90',       [pi/2; 0; 0; 0; 0; 0; 0];
    'J2=90',       [0; pi/2; 0; 0; 0; 0; 0];
    'J3=90',       [0; 0; pi/2; 0; 0; 0; 0];
    '전체45',      ones(7,1)*pi/4;
    '혼합',        [pi/6; -pi/4; pi/3; -pi/6; pi/4; -pi/3; pi/6];
};

fprintf('=== DH FK vs URDF FK 비교 ===\n');
fprintf('%-12s | %-30s | %-30s | %s\n', '구성', 'DH EE [mm]', 'URDF EE [mm]', '오차[mm]');
fprintf('------------------------------------------------------------------------------------------------\n');

for c = 1:size(test_configs, 1)
    q = test_configs{c, 2};
    
    %% DH FK 계산
    T_dh = eye(4);
    for i = 1:7
        a = DH(i,1);
        alpha = DH(i,2);
        d = DH(i,3);
        theta = q(i) + DH(i,4);
        T_dh = T_dh * dh_transform(a, alpha, d, theta);
    end
    p_dh = T_dh(1:3, 4) * 1000; % mm
    
    %% URDF FK 계산
    T_urdf = getTransform(robot, q, 'eef_link');
    p_urdf = T_urdf(1:3, 4) * 1000; % mm
    
    %% 오차
    err = norm(p_dh - p_urdf);
    
    fprintf('%-12s | [%8.1f, %8.1f, %8.1f] | [%8.1f, %8.1f, %8.1f] | %8.2f\n', ...
        test_configs{c,1}, p_dh, p_urdf, err);
end

%% 상세 비교 - 영점에서 각 프레임
fprintf('\n=== 영점에서 각 프레임 위치 비교 ===\n');
q = zeros(7,1);

fprintf('%-10s | %-25s | %-25s | %s\n', '프레임', 'DH [mm]', 'URDF [mm]', '오차');
fprintf('--------------------------------------------------------------------------------\n');

link_names = {'link1', 'link2', 'link3', 'link4', 'link5', 'link6', 'link7', 'eef_link'};

T_dh = eye(4);
for i = 1:7
    a = DH(i,1); alpha = DH(i,2); d = DH(i,3); theta = q(i) + DH(i,4);
    T_dh = T_dh * dh_transform(a, alpha, d, theta);
    p_dh = T_dh(1:3,4) * 1000;
    
    T_urdf = getTransform(robot, q, link_names{i});
    p_urdf = T_urdf(1:3,4) * 1000;
    
    err = norm(p_dh - p_urdf);
    fprintf('%-10s | [%7.1f, %7.1f, %7.1f] | [%7.1f, %7.1f, %7.1f] | %7.1f\n', ...
        link_names{i}, p_dh, p_urdf, err);
end

% EEF
T_urdf = getTransform(robot, q, 'eef_link');
p_urdf = T_urdf(1:3,4) * 1000;
p_dh = T_dh(1:3,4) * 1000; % 마지막 DH 결과
err = norm(p_dh - p_urdf);
fprintf('%-10s | [%7.1f, %7.1f, %7.1f] | [%7.1f, %7.1f, %7.1f] | %7.1f\n', ...
    'eef_link', p_dh, p_urdf, err);

%% 시각화 비교
figure('Name', 'DH vs URDF 시각화', 'Position', [100,100,1400,600], 'Color', 'w');

% URDF 시각화
subplot(1,2,1);
show(robot, zeros(7,1), 'Visuals', 'off', 'Frames', 'on');
title('URDF 모델 (영점)');
view(45, 25); grid on; axis equal;
xlabel('X'); ylabel('Y'); zlabel('Z');

% DH 기반 프레임 위치
subplot(1,2,2);
T = eye(4);
positions_dh = zeros(8, 3);
positions_dh(1,:) = [0, 0, 0];

for i = 1:7
    a = DH(i,1); alpha = DH(i,2); d = DH(i,3); theta = DH(i,4);
    T = T * dh_transform(a, alpha, d, theta);
    positions_dh(i+1,:) = T(1:3,4)';
end

plot3(positions_dh(:,1), positions_dh(:,2), positions_dh(:,3), ...
    'bo-', 'LineWidth', 3, 'MarkerSize', 12, 'MarkerFaceColor', 'b');
hold on;

% URDF 위치 오버레이
urdf_pos = zeros(9, 3);
link_all = {'base', 'link1', 'link2', 'link3', 'link4', 'link5', 'link6', 'link7', 'eef_link'};
for i = 1:9
    T_urdf = getTransform(robot, zeros(7,1), link_all{i});
    urdf_pos(i,:) = T_urdf(1:3,4)';
end
plot3(urdf_pos(:,1), urdf_pos(:,2), urdf_pos(:,3), ...
    'rs--', 'LineWidth', 2, 'MarkerSize', 10, 'MarkerFaceColor', 'r');

legend('DH FK', 'URDF FK');
title('DH vs URDF 프레임 위치 비교');
view(45, 25); grid on; axis equal;
xlabel('X[m]'); ylabel('Y[m]'); zlabel('Z[m]');

fprintf('\n=== 비교 완료 ===\n');