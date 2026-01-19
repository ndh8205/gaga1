%% check_stl_size.m
% STL 모델 크기 확인
clear all
close all
clc

% stl_path = 'D:\pj2025\space_challenge\model\modeling_3d\Chaser_v2.stl';
stl_path = 'D:\pj2025\space_challenge\model\modeling_3d\Chaser_v2_converted.stl';
% stl_path = 'D:\pj2025\space_challenge\model\modeling_3d\client2.stl';

tr = stlread(stl_path);
v = tr.Points;

% 바운딩 박스
x_range = [min(v(:,1)), max(v(:,1))];
y_range = [min(v(:,2)), max(v(:,2))];
z_range = [min(v(:,3)), max(v(:,3))];

fprintf('=== STL 크기 분석 ===\n');
fprintf('X: %.4f ~ %.4f (크기: %.4f)\n', x_range, diff(x_range));
fprintf('Y: %.4f ~ %.4f (크기: %.4f)\n', y_range, diff(y_range));
fprintf('Z: %.4f ~ %.4f (크기: %.4f)\n', z_range, diff(z_range));
fprintf('\n단위가 mm면 1000으로 나눠야 함\n');

% 시각화
figure;
trisurf(tr, 'FaceColor', [0.5 0.5 0.6], 'EdgeColor', 'none');
axis equal; lighting gouraud; light;
xlabel('X'); ylabel('Y'); zlabel('Z');
title(sprintf('Size: %.2f x %.2f x %.2f', diff(x_range), diff(y_range), diff(z_range)));

%% convert_stl_coords.m
% STL 좌표계 변환: Z→X, -Y→Y, X→Z + 무게중심 정중앙으로
% 
stl_in = 'D:\pj2025\space_challenge\model\modeling_3d\Chaser_v2.stl';
stl_out = 'D:\pj2025\space_challenge\model\modeling_3d\Chaser_v2_converted.stl';

% stl_in = 'D:\pj2025\space_challenge\model\modeling_3d\client2.stl';
% stl_out = 'D:\pj2025\space_challenge\model\modeling_3d\client_converted.stl';

tr = stlread(stl_in);
v = tr.Points;

% 변환: 새X=Z, 새Y=-Y, 새Z=X
v_new = [v(:,3), -v(:,2), v(:,1)];
% v_new = v_new / 1000;


% 바운딩 박스 중심 계산
center = [(max(v_new(:,1)) + min(v_new(:,1)))/2, ...
          (max(v_new(:,2)) + min(v_new(:,2)))/2, ...
          (max(v_new(:,3)) + min(v_new(:,3)))/2];

fprintf('변환 전 중심: [%.3f, %.3f, %.3f]\n', center);

% 중심을 원점으로 이동
v_new = v_new - center;

% 새 triangulation
tr_new = triangulation(tr.ConnectivityList, v_new);

% 저장
stlwrite(tr_new, stl_out);

% 확인
fprintf('변환 완료: %s\n', stl_out);
fprintf('새 크기:\n');
fprintf('  X: %.2f ~ %.2f (%.2f)\n', min(v_new(:,1)), max(v_new(:,1)), max(v_new(:,1))-min(v_new(:,1)));
fprintf('  Y: %.2f ~ %.2f (%.2f)\n', min(v_new(:,2)), max(v_new(:,2)), max(v_new(:,2))-min(v_new(:,2)));
fprintf('  Z: %.2f ~ %.2f (%.2f)\n', min(v_new(:,3)), max(v_new(:,3)), max(v_new(:,3))-min(v_new(:,3)));

% 시각화
figure;
trisurf(tr_new, 'FaceColor', [0.5 0.5 0.6], 'EdgeColor', 'none', 'FaceAlpha', 0.7);
hold on;
axis equal; lighting gouraud; light;

% 중앙 좌표축 (길이 3m)
L = max([diff(x_range), diff(y_range), diff(z_range)]) /1000 * 0.3;
quiver3(0, 0, 0, L, 0, 0, 0, 'r', 'LineWidth', 3, 'MaxHeadSize', 0.3);
quiver3(0, 0, 0, 0, L, 0, 0, 'g', 'LineWidth', 3, 'MaxHeadSize', 0.3);
quiver3(0, 0, 0, 0, 0, L, 0, 'b', 'LineWidth', 3, 'MaxHeadSize', 0.3);
text(L*1.1, 0, 0, 'X (전방)', 'Color', 'r', 'FontSize', 12, 'FontWeight', 'bold');
text(0, L*1.1, 0, 'Y (우측)', 'Color', 'g', 'FontSize', 12, 'FontWeight', 'bold');
text(0, 0, L*1.1, 'Z (하방)', 'Color', 'b', 'FontSize', 12, 'FontWeight', 'bold');

% 무게중심 마커 (원점)
plot3(0, 0, 0, 'mo', 'MarkerSize', 15, 'LineWidth', 3, 'MarkerFaceColor', 'm');
text(0.3, 0.3, 0.3, 'CoM', 'Color', 'm', 'FontSize', 14, 'FontWeight', 'bold');

xlabel('X'); ylabel('Y'); zlabel('Z');
title('변환된 좌표계 (무게중심 = 원점)');
view(45, 25);
grid on;


%% visualize_dual_arm.m
% 위성 + 양쪽 로봇팔 시각화 (위성 꾸미기)
clear;

%% 경로
stl_sat = 'D:\pj2025\space_challenge\model\modeling_3d\Chaser_v2_converted.stl';
meshPath = 'D:\pj2025\space_challenge\model\modeling_3d\ASM_KARI_ARM\meshes';

%% 위성 물리 파라미터
sat.m = 500;  % [kg]
sat.I = [260, -0.2, 0.6; -0.2, 280, 4; 0.6, 4, 170];  % [kg·m²]

%% 장착점
mount_Y_pos = [1.9, 0.9, 0];   % +Y 팔
mount_Y_neg = [1.9, -0.9, 0];  % -Y 팔

%% 위성 본체 로드
tr_sat = stlread(stl_sat);
v_sat = tr_sat.Points;
f_sat = tr_sat.ConnectivityList;

sat.size = [max(v_sat(:,1))-min(v_sat(:,1)), ...
            max(v_sat(:,2))-min(v_sat(:,2)), ...
            max(v_sat(:,3))-min(v_sat(:,3))];
sat.body_size = [2.4, 2.4, 3.95];

%% 정보 출력
fprintf('============================================================\n');
fprintf('  위성 + 듀얼 로봇팔 시스템 정보\n');
fprintf('============================================================\n');
fprintf('\n[ 위성 (Chaser) ]\n');
fprintf('  전체 크기 (태양전지판 포함): %.2f x %.2f x %.2f m\n', sat.size);
fprintf('  본체 크기: %.2f x %.2f x %.2f m\n', sat.body_size);
fprintf('  질량: %.0f kg\n', sat.m);
fprintf('  관성 텐서 [kg·m²]:\n');
fprintf('    [%7.1f %7.1f %7.1f]\n', sat.I(1,:));
fprintf('    [%7.1f %7.1f %7.1f]\n', sat.I(2,:));
fprintf('    [%7.1f %7.1f %7.1f]\n', sat.I(3,:));
fprintf('\n[ 로봇팔 장착점 ]\n');
fprintf('  +Y 팔: [%.2f, %.2f, %.2f] m\n', mount_Y_pos);
fprintf('  -Y 팔: [%.2f, %.2f, %.2f] m\n', mount_Y_neg);
fprintf('  CoM에서 거리: %.3f m\n', norm(mount_Y_pos));
fprintf('\n[ 좌표계 정의 ]\n');
fprintf('  X: 전방 (도킹 방향)\n');
fprintf('  Y: 우측 (로봇팔 전개 방향)\n');
fprintf('  Z: 하방 (나디르)\n');
fprintf('============================================================\n');

%% 그림 설정
figure('Color', 'k', 'Position', [100 100 1400 900]);
ax = axes('Color', 'k', 'XColor', 'w', 'YColor', 'w', 'ZColor', 'w'); 
hold on;
axis equal; grid on;
ax.GridColor = [0.3 0.3 0.3];
xlabel('X (전방)'); ylabel('Y (우측)'); zlabel('Z (하방)');
view(45, 25);

%% 위성 면 분류
face_centers = zeros(size(f_sat, 1), 3);
for i = 1:size(f_sat, 1)
    face_centers(i, :) = mean(v_sat(f_sat(i, :), :), 1);
end

% 본체 영역 threshold
body_threshold_X = 1.3;
body_threshold_Y = 1.3;
body_threshold_Z = 1.5;

% 본체: 중앙 박스 영역
is_body = abs(face_centers(:, 1)) < body_threshold_X & ...
          abs(face_centers(:, 2)) < body_threshold_Y & ...
          abs(face_centers(:, 3)) < body_threshold_Z;

% 전방 돌출부 (안테나/센서): X > threshold
is_front = face_centers(:, 1) > body_threshold_X;

% 태양전지판: Y 또는 Z 외곽
is_panel = abs(face_centers(:, 2)) >= body_threshold_Y | ...
           abs(face_centers(:, 3)) >= body_threshold_Z;
is_panel = is_panel & ~is_front;

%% 색상 정의
color_body = [0.85, 0.65, 0.13];  % 금색 (MLI)
color_panel = [0.1, 0.2, 0.5];    % 태양전지판
color_front = [0.95, 0.95, 0.95]; % 전방 돌출부 (흰색)

% 면별 색상 배열
face_colors = zeros(size(f_sat, 1), 3);
face_colors(is_body, :) = repmat(color_body, sum(is_body), 1);
face_colors(is_panel, :) = repmat(color_panel, sum(is_panel), 1);
face_colors(is_front, :) = repmat(color_front, sum(is_front), 1);

%% 위성 그리기
patch('Faces', f_sat, 'Vertices', v_sat, ...
    'FaceVertexCData', face_colors, ...
    'FaceColor', 'flat', ...
    'EdgeColor', 'none', ...
    'FaceLighting', 'gouraud', ...
    'AmbientStrength', 0.4, ...
    'DiffuseStrength', 0.6, ...
    'SpecularStrength', 0.8, ...
    'SpecularExponent', 25);

%% 로봇팔 정의
robotDef = {
    'ASM_J0.STL', [0 0 0];
    'ASM_J1.STL', [0 0 0.094];
    'ASM_J2.STL', [0 0.088 0.105];
    'ASM_J3.STL', [0 0.104 0.131];
    'ASM_J4.STL', [0 -0.088 0.8];
    'ASM_J5.STL', [0 -0.104 0.131];
    'ASM_J6.STL', [0 -0.071 0.408];
    'ASM_J7.STL', [0 -0.121 0.088];
};

%% 로봇팔 그리기
R_pos = [1 0 0; 0 0 1; 0 -1 0];  
draw_robot_arm(ax, meshPath, robotDef, mount_Y_pos, R_pos, [0.9 0.9 0.95]);

R_neg = [1 0 0; 0 0 -1; 0 1 0];
draw_robot_arm(ax, meshPath, robotDef, mount_Y_neg, R_neg, [0.9 0.9 0.95]);

%% 조명 (우주 환경)
light('Position', [10 5 -5], 'Style', 'infinite', 'Color', [1 1 0.95]);
light('Position', [-5 -5 5], 'Style', 'infinite', 'Color', [0.1 0.15 0.3]);
light('Position', [0 0 -10], 'Style', 'infinite', 'Color', [0.05 0.05 0.1]);

%% 좌표축 (중앙)
L = 3;
quiver3(0, 0, 0, L, 0, 0, 0, 'r', 'LineWidth', 3, 'MaxHeadSize', 0.3);
quiver3(0, 0, 0, 0, L, 0, 0, 'g', 'LineWidth', 3, 'MaxHeadSize', 0.3);
quiver3(0, 0, 0, 0, 0, L, 0, 'b', 'LineWidth', 3, 'MaxHeadSize', 0.3);
text(L*1.1, 0, 0, 'X', 'Color', 'r', 'FontSize', 12, 'FontWeight', 'bold');
text(0, L*1.1, 0, 'Y', 'Color', 'g', 'FontSize', 12, 'FontWeight', 'bold');
text(0, 0, L*1.1, 'Z', 'Color', 'b', 'FontSize', 12, 'FontWeight', 'bold');

%% 무게중심
plot3(0, 0, 0, 'mo', 'MarkerSize', 12, 'LineWidth', 2, 'MarkerFaceColor', 'm');

%% 장착점 글씨만
text(mount_Y_pos(1)+0.3, mount_Y_pos(2)+0.3, mount_Y_pos(3), '+Y Arm', 'Color', 'w', 'FontSize', 10);
text(mount_Y_neg(1)+0.3, mount_Y_neg(2)-0.5, mount_Y_neg(3), '-Y Arm', 'Color', 'w', 'FontSize', 10);

%% 별 배경
n_stars = 200;
star_range = 15;
stars_x = (rand(n_stars, 1) - 0.5) * star_range * 2;
stars_y = (rand(n_stars, 1) - 0.5) * star_range * 2;
stars_z = (rand(n_stars, 1) - 0.5) * star_range * 2;
star_size = rand(n_stars, 1) * 10 + 2;
scatter3(stars_x, stars_y, stars_z, star_size, 'w', 'filled', 'MarkerFaceAlpha', 0.6);

title('Dual-Arm Space Servicer', 'Color', 'w', 'FontSize', 14);
xlim([-8 8]); ylim([-8 8]); zlim([-6 6]);

%% ========== 로봇팔 그리기 함수 ==========
function draw_robot_arm(ax, meshPath, robotDef, mount, R_mount, color)
    pos = mount(:);
    R = R_mount;

    for k = 1:size(robotDef, 1)
        fileName = robotDef{k, 1};
        offset = robotDef{k, 2}(:);
        fullPath = fullfile(meshPath, fileName);

        pos = pos + R * offset;

        if isfile(fullPath)
            tr = stlread(fullPath);
            v = tr.Points;
            v_transformed = (R * v')' + pos';

            patch('Parent', ax, ...
                'Faces', tr.ConnectivityList, ...
                'Vertices', v_transformed, ...
                'FaceColor', color, ...
                'EdgeColor', 'none', ...
                'FaceLighting', 'gouraud', ...
                'AmbientStrength', 0.3, ...
                'DiffuseStrength', 0.7, ...
                'SpecularStrength', 0.5);
        end
    end
end

% %% visualize_client.m
% % Client 위성 시각화 (타겟)
% clear;
% 
% %% 경로
% stl_sat = 'D:\pj2025\space_challenge\model\modeling_3d\client_converted.stl';
% 
% %% 위성 본체 로드
% tr_sat = stlread(stl_sat);
% v_sat = tr_sat.Points;
% f_sat = tr_sat.ConnectivityList;
% 
% sat.size = [max(v_sat(:,1))-min(v_sat(:,1)), ...
%             max(v_sat(:,2))-min(v_sat(:,2)), ...
%             max(v_sat(:,3))-min(v_sat(:,3))];
% 
% %% 정보 출력
% fprintf('============================================================\n');
% fprintf('  Client 위성 (Target) 정보\n');
% fprintf('============================================================\n');
% fprintf('  전체 크기: %.2f x %.2f x %.2f m\n', sat.size);
% fprintf('\n[ 좌표계 정의 ]\n');
% fprintf('  X: 전방\n');
% fprintf('  Y: 우측 (태양전지판 방향)\n');
% fprintf('  Z: 하방\n');
% fprintf('============================================================\n');
% 
% %% 그림 설정
% figure('Color', 'k', 'Position', [100 100 1400 900]);
% ax = axes('Color', 'k', 'XColor', 'w', 'YColor', 'w', 'ZColor', 'w'); 
% hold on;
% axis equal; grid on;
% ax.GridColor = [0.3 0.3 0.3];
% xlabel('X (전방)'); ylabel('Y (우측)'); zlabel('Z (하방)');
% view(45, 25);
% 
% %% 위성 면 분류
% face_centers = zeros(size(f_sat, 1), 3);
% for i = 1:size(f_sat, 1)
%     face_centers(i, :) = mean(v_sat(f_sat(i, :), :), 1);
% end
% 
% % 본체 영역 threshold (크기 기반 adaptive)
% body_threshold_X = sat.size(1) * 0.4;
% body_threshold_Y = sat.size(2) * 0.3;
% body_threshold_Z = sat.size(3) * 0.4;
% 
% % 본체: 중앙 박스 영역
% is_body = abs(face_centers(:, 1)) < body_threshold_X & ...
%           abs(face_centers(:, 2)) < body_threshold_Y & ...
%           abs(face_centers(:, 3)) < body_threshold_Z;
% 
% % 전방 돌출부 (안테나/센서): X > threshold
% is_front = face_centers(:, 1) > body_threshold_X;
% 
% % 태양전지판: Y 외곽
% is_panel = abs(face_centers(:, 2)) >= body_threshold_Y;
% is_panel = is_panel & ~is_front;
% 
% %% 색상 정의
% color_body = [0.85, 0.65, 0.13];  % 금색 (MLI)
% color_panel = [0.1, 0.2, 0.5];    % 태양전지판
% color_front = [0.95, 0.95, 0.95]; % 전방 돌출부 (흰색)
% 
% % 면별 색상 배열
% face_colors = zeros(size(f_sat, 1), 3);
% face_colors(is_body, :) = repmat(color_body, sum(is_body), 1);
% face_colors(is_panel, :) = repmat(color_panel, sum(is_panel), 1);
% face_colors(is_front, :) = repmat(color_front, sum(is_front), 1);
% 
% % 미분류 면 기본색 (금색)
% is_none = ~is_body & ~is_panel & ~is_front;
% face_colors(is_none, :) = repmat(color_body, sum(is_none), 1);
% 
% %% 위성 그리기
% patch('Faces', f_sat, 'Vertices', v_sat, ...
%     'FaceVertexCData', face_colors, ...
%     'FaceColor', 'flat', ...
%     'EdgeColor', 'none', ...
%     'FaceLighting', 'gouraud', ...
%     'AmbientStrength', 0.4, ...
%     'DiffuseStrength', 0.6, ...
%     'SpecularStrength', 0.8, ...
%     'SpecularExponent', 25);
% 
% %% 조명 (우주 환경)
% light('Position', [10 5 -5], 'Style', 'infinite', 'Color', [1 1 0.95]);
% light('Position', [-5 -5 5], 'Style', 'infinite', 'Color', [0.1 0.15 0.3]);
% light('Position', [0 0 -10], 'Style', 'infinite', 'Color', [0.05 0.05 0.1]);
% 
% %% 좌표축 (adaptive)
% L = max(sat.size) * 0.4;
% quiver3(0, 0, 0, L, 0, 0, 0, 'r', 'LineWidth', 3, 'MaxHeadSize', 0.3);
% quiver3(0, 0, 0, 0, L, 0, 0, 'g', 'LineWidth', 3, 'MaxHeadSize', 0.3);
% quiver3(0, 0, 0, 0, 0, L, 0, 'b', 'LineWidth', 3, 'MaxHeadSize', 0.3);
% text(L*1.1, 0, 0, 'X', 'Color', 'r', 'FontSize', 12, 'FontWeight', 'bold');
% text(0, L*1.1, 0, 'Y', 'Color', 'g', 'FontSize', 12, 'FontWeight', 'bold');
% text(0, 0, L*1.1, 'Z', 'Color', 'b', 'FontSize', 12, 'FontWeight', 'bold');
% 
% %% 무게중심
% plot3(0, 0, 0, 'mo', 'MarkerSize', 12, 'LineWidth', 2, 'MarkerFaceColor', 'm');
% text(L*0.15, L*0.15, L*0.15, 'CoM', 'Color', 'm', 'FontSize', 12, 'FontWeight', 'bold');
% 
% %% 별 배경
% n_stars = 200;
% star_range = max(sat.size) * 3;
% stars_x = (rand(n_stars, 1) - 0.5) * star_range * 2;
% stars_y = (rand(n_stars, 1) - 0.5) * star_range * 2;
% stars_z = (rand(n_stars, 1) - 0.5) * star_range * 2;
% star_size = rand(n_stars, 1) * 10 + 2;
% scatter3(stars_x, stars_y, stars_z, star_size, 'w', 'filled', 'MarkerFaceAlpha', 0.6);
% 
% title('Client Satellite (Target)', 'Color', 'w', 'FontSize', 14);
% 
% lim = max(sat.size) * 1.5;
% xlim([-lim lim]); ylim([-lim lim]); zlim([-lim lim]);