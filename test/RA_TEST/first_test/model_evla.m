%% KARI ARM 시각화 - 링크 길이 및 CG 표시
clear all; close all; clc;

%% 경로 설정
base_path = 'D:\hanul2026_scie\mppi_RA';
addpath(genpath(base_path));

%% URDF 로드
urdf_files = dir(fullfile(base_path, '**', '*KARI*.urdf'));
robot = importrobot(fullfile(urdf_files(1).folder, urdf_files(1).name), 'DataFormat', 'column');
robot.Gravity = [0 0 -9.81];
dof = numel(homeConfiguration(robot));
fprintf('KARI ARM 로드: %d DOF\n', dof);

%% 관절 원점 정보 추출 (URDF 기준)
joint_origins = [
    0,      0,       0.094;    % J1: base -> link1
    0,      0.088,   0.105;    % J2: link1 -> link2
    0,      0.104,   0.131;    % J3: link2 -> link3
    0,     -0.088,   0.800;    % J4: link3 -> link4
    0,     -0.104,   0.131;    % J5: link4 -> link5
    0,     -0.071,   0.408;    % J6: link5 -> link6
    0,     -0.121,   0.088;    % J7: link6 -> link7
    0,      0,       0.097;    % EEF: link7 -> eef
];

link_names = {'base', 'link1', 'link2', 'link3', 'link4', 'link5', 'link6', 'link7', 'eef_link'};

%% 링크 길이 계산
fprintf('\n=== 링크 길이 (관절 간 거리) ===\n');
fprintf('%-15s | %8s | %8s | %8s | %8s\n', '구간', 'dX[mm]', 'dY[mm]', 'dZ[mm]', '거리[mm]');
fprintf('--------------------------------------------------------------\n');
total_z = 0;
for i = 1:size(joint_origins, 1)
    dx = joint_origins(i,1) * 1000;
    dy = joint_origins(i,2) * 1000;
    dz = joint_origins(i,3) * 1000;
    dist = norm(joint_origins(i,:)) * 1000;
    total_z = total_z + joint_origins(i,3);
    fprintf('%-15s | %8.1f | %8.1f | %8.1f | %8.1f\n', ...
        sprintf('%s->%s', link_names{i}, link_names{i+1}), dx, dy, dz, dist);
end
fprintf('--------------------------------------------------------------\n');
fprintf('%-15s | %8s | %8s | %8.1f | \n', '총 Z 높이', '', '', total_z*1000);

%% 시각화 - 영점 구성
q = zeros(dof, 1);

figure('Name', 'KARI ARM 구조 분석', 'Position', [50, 50, 1600, 800], 'Color', 'w');

%% 1. 3D 뷰 + CG 표시
subplot(1,2,1);
show(robot, q, 'Visuals', 'off', 'Frames', 'on');
hold on;

% 각 링크의 글로벌 CG 위치 계산 및 표시
cg_global = zeros(length(robot.Bodies), 3);
for i = 1:length(robot.Bodies)
    body = robot.Bodies{i};
    if body.Mass > 0
        % 링크 프레임의 글로벌 변환
        T = getTransform(robot, q, body.Name);
        % CG의 글로벌 위치
        com_local = body.CenterOfMass(:);
        cg_global(i,:) = (T(1:3,1:3) * com_local + T(1:3,4))';
        
        % CG 점 표시
        plot3(cg_global(i,1), cg_global(i,2), cg_global(i,3), ...
            'ro', 'MarkerSize', 12, 'MarkerFaceColor', 'r', 'LineWidth', 2);
        
        % CG 라벨
        text(cg_global(i,1)+0.03, cg_global(i,2), cg_global(i,3), ...
            sprintf('%s\n%.2fkg', body.Name, body.Mass), ...
            'FontSize', 8, 'Color', 'r');
    end
end

% 관절 위치 계산 및 연결선
joint_pos = zeros(length(link_names), 3);
joint_pos(1,:) = [0, 0, 0]; % base
for i = 1:length(link_names)-1
    T = getTransform(robot, q, link_names{i+1});
    joint_pos(i+1,:) = T(1:3,4)';
end

% 관절 연결선 (파란색)
plot3(joint_pos(:,1), joint_pos(:,2), joint_pos(:,3), ...
    'b-', 'LineWidth', 3);
plot3(joint_pos(:,1), joint_pos(:,2), joint_pos(:,3), ...
    'bo', 'MarkerSize', 8, 'MarkerFaceColor', 'b');

title('KARI ARM - 3D 구조 (빨강: CG, 파랑: 관절)');
xlabel('X [m]'); ylabel('Y [m]'); zlabel('Z [m]');
view(45, 25); grid on; axis equal;
legend('프레임', 'CG', '', '관절 연결', '관절점', 'Location', 'best');

%% 2. 측면도 (Y-Z 평면) + 치수 표시
subplot(1,2,2);
show(robot, q, 'Visuals', 'off', 'Frames', 'off');
hold on;

% 관절점 표시
plot3(joint_pos(:,1), joint_pos(:,2), joint_pos(:,3), ...
    'bo-', 'LineWidth', 2, 'MarkerSize', 10, 'MarkerFaceColor', 'b');

% CG 표시
for i = 1:length(robot.Bodies)
    if robot.Bodies{i}.Mass > 0
        plot3(cg_global(i,1), cg_global(i,2), cg_global(i,3), ...
            'rs', 'MarkerSize', 10, 'MarkerFaceColor', 'r');
    end
end

% 치수선 표시 (Z 방향)
z_offset = 0.15;
for i = 1:size(joint_origins, 1)
    if joint_origins(i,3) > 0.05  % 의미있는 Z 변위만
        z1 = sum(joint_origins(1:i-1, 3));
        z2 = sum(joint_origins(1:i, 3));
        y_pos = joint_pos(i,2) - z_offset;
        
        % 치수선
        plot3([z_offset, z_offset], [y_pos, y_pos], [z1, z2], 'k-', 'LineWidth', 1.5);
        plot3([z_offset-0.02, z_offset+0.02], [y_pos, y_pos], [z1, z1], 'k-', 'LineWidth', 1.5);
        plot3([z_offset-0.02, z_offset+0.02], [y_pos, y_pos], [z2, z2], 'k-', 'LineWidth', 1.5);
        
        % 치수 텍스트
        text(z_offset+0.03, y_pos, (z1+z2)/2, ...
            sprintf('%.0fmm', joint_origins(i,3)*1000), ...
            'FontSize', 9, 'FontWeight', 'bold');
    end
end

% 총 높이 표시
plot3([0.25, 0.25], [0, 0], [0, total_z], 'r-', 'LineWidth', 2);
text(0.28, 0, total_z/2, sprintf('총 높이\n%.0fmm', total_z*1000), ...
    'FontSize', 10, 'FontWeight', 'bold', 'Color', 'r');

title('KARI ARM - 측면도 + 치수');
xlabel('X [m]'); ylabel('Y [m]'); zlabel('Z [m]');
view(0, 0); grid on; axis equal;

%% 링크별 상세 정보 출력
fprintf('\n=== 링크별 상세 정보 ===\n');
fprintf('%-10s | %8s | %-25s | %-35s\n', '링크', '질량[kg]', 'CoM(로컬)[mm]', '관성[kg·m²] Ixx,Iyy,Izz');
fprintf('----------------------------------------------------------------------------------------\n');
for i = 1:length(robot.Bodies)
    body = robot.Bodies{i};
    com = body.CenterOfMass * 1000;
    inertia = body.Inertia;
    fprintf('%-10s | %8.3f | [%7.1f, %7.1f, %7.1f] | [%.4f, %.4f, %.4f]\n', ...
        body.Name, body.Mass, com(1), com(2), com(3), inertia(1), inertia(2), inertia(3));
end

%% 전체 시스템 CG 계산
total_mass = 0;
system_cg = [0; 0; 0];
for i = 1:length(robot.Bodies)
    m = robot.Bodies{i}.Mass;
    total_mass = total_mass + m;
    system_cg = system_cg + m * cg_global(i,:)';
end
system_cg = system_cg / total_mass;

fprintf('\n=== 시스템 전체 ===\n');
fprintf('총 질량: %.3f kg\n', total_mass);
fprintf('시스템 CG (영점): [%.4f, %.4f, %.4f] m\n', system_cg);

%% 추가: 정면도
figure('Name', 'KARI ARM 다중 뷰', 'Position', [100, 100, 1400, 500], 'Color', 'w');

views = {[0, 0], '측면 (Y-Z)'; [90, 0], '정면 (X-Z)'; [0, 90], '상면 (X-Y)'};
for v = 1:3
    subplot(1,3,v);
    show(robot, q, 'Visuals', 'off', 'Frames', 'on');
    hold on;
    
    % 관절 연결
    plot3(joint_pos(:,1), joint_pos(:,2), joint_pos(:,3), ...
        'b-', 'LineWidth', 2);
    
    % CG 표시
    for i = 1:length(robot.Bodies)
        if robot.Bodies{i}.Mass > 0
            plot3(cg_global(i,1), cg_global(i,2), cg_global(i,3), ...
                'ro', 'MarkerSize', 8, 'MarkerFaceColor', 'r');
        end
    end
    
    % 시스템 CG
    plot3(system_cg(1), system_cg(2), system_cg(3), ...
        'gd', 'MarkerSize', 15, 'MarkerFaceColor', 'g', 'LineWidth', 2);
    
    title(views{v,2});
    view(views{v,1});
    grid on; axis equal;
    xlabel('X'); ylabel('Y'); zlabel('Z');
end

fprintf('\n=== 시각화 완료 ===\n');