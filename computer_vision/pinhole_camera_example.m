clc; clear all; close all;
%% 시스템 파라미터 초기화
deg = pi/180;
rad = 180/pi;

% 색상 정의 (RGB)
colors = struct('x_axis', [1, 0, 0], ...      % 빨강 (X축)
                'y_axis', [0, 1, 0], ...      % 초록 (Y축) 
                'z_axis', [0, 0, 1], ...      % 파랑 (Z축)
                'camera', [0.8, 0.2, 0.2], ... % 카메라
                'world_obj', [1, 0.6, 0], ...  % 3D 객체
                'image_proj', [0.2, 0.4, 0.8]);% 2D 투영점

%% 카메라 내부 파라미터 (Intrinsic Parameters)
image_width = 1024;
image_height = 1024;
fov = 20 * deg; % 60도 시야각
% fov = 0.1; % 60도 시야각


% 초점거리 (Focal Length)
f = image_width / (2 * tan(fov/2));
f_x = f;
f_y = f;

% 주점 (Principal Point)
c_x = image_width / 2;
c_y = image_height / 2;

% 카메라 내부 파라미터 행렬 K
K = [f_x,  0,  c_x;
     0,  f_y,  c_y;
     0,   0,    1];

fprintf('=== Pinhole Camera Model Simplified ===\n');
fprintf('카메라 내부 파라미터 (K):\n');
disp(K);

%% 3D 월드 객체 생성
% 월드 좌표계 중심에 3x3x3 크기의 큐브 생성
world_points = GenerateCube(3, [0; 0; 0]);
n_points = size(world_points, 2);

%% 카메라 외부 파라미터 (Extrinsic Parameters)
% 카메라 위치 (Position)
camera_pos = [10; -8; 5]; 

% 카메라가 바라보는 지점 (Target)
target = [0; 0; 0];

% 카메라의 회전 행렬 (Rotation Matrix) 계산
% 월드 좌표계 -> 카메라 좌표계 변환 행렬의 회전 부분
R_C_W = LookAtTarget(target, camera_pos);

fprintf('카메라 위치: [%.1f, %.1f, %.1f]\n', camera_pos);

%% 3D 포인트를 2D 이미지 평면에 투영
image_pixels = zeros(2, n_points);
is_visible = false(1, n_points);

for i = 1:n_points
    [image_pixels(:, i), is_visible(i)] = ProjectPoint(world_points(:, i), K, R_C_W, camera_pos, image_width, image_height);
end

%% 시각화
fig = figure('Name', 'Pinhole Camera Projection', 'Position', [100, 100, 1200, 600]);

% --- 서브플롯 1: 3D 월드 뷰 ---
subplot(1, 2, 1);
cla; hold on;

DrawWorldAxes(5, colors);
DrawCube3D(world_points, colors.world_obj);
DrawCamera(inv(R_C_W), camera_pos, 2, colors); % 카메라 자세는 R_W_C를 사용

% 카메라와 각 정점을 잇는 투영선 그리기
for i = 1:n_points
    if is_visible(i)
        plot3([camera_pos(1), world_points(1,i)], ...
              [camera_pos(2), world_points(2,i)], ...
              [camera_pos(3), world_points(3,i)], ...
              ':', 'Color', [0.5 0.5 0.5]);
    end
end

hold off;
axis equal; grid on;
xlim([-10, 15]); ylim([-15, 10]); zlim([-5, 10]);
xlabel('X [m]'); ylabel('Y [m]'); zlabel('Z [m]');
title('3D World View', 'FontSize', 12);
view(35, 25);

% --- 서브플롯 2: 2D 카메라 뷰 ---
subplot(1, 2, 2);
cla; hold on;

% 이미지 경계
rectangle('Position', [0, 0, image_width, image_height], 'EdgeColor', 'k', 'LineWidth', 2);

% 투영된 큐브 그리기
DrawCube2D(image_pixels, is_visible, colors.image_proj);

% 주축점 표시
plot(c_x, c_y, 'r+', 'MarkerSize', 15, 'LineWidth', 2);

hold off;
xlim([-50, image_width+50]); ylim([-50, image_height+50]);
set(gca, 'YDir', 'reverse'); % 이미지 좌표계는 y축이 아래로 향함
axis equal; grid on;
xlabel('u [pixels]'); ylabel('v [pixels]');
title('Camera View (2D Projection)', 'FontSize', 12);

sgtitle('Pinhole Camera Model and Projection', 'FontSize', 16, 'FontWeight', 'bold');

fprintf('\n=== Visualization Complete ===\n');

%% --- 함수 정의 ---

function [pixel, is_visible] = ProjectPoint(X_world, K, R_C_W, t_W_C, img_w, img_h)
    % 3D 월드 좌표(X_world)를 2D 픽셀 좌표(pixel)로 투영하는 함수
    
    % 1. 월드 좌표계 -> 카메라 좌표계 변환
    t_C_W = -R_C_W * t_W_C; % 카메라 좌표계에서 본 월드의 원점
    X_cam = R_C_W * X_world + t_C_W;
    
    % 2. 카메라 앞에 있는지 확인 (z > 0)
    if X_cam(3) <= 0.1 % 0.1은 클리핑 평면 역할
        pixel = [-1; -1];
        is_visible = false;
        return;
    end
    
    % 3. 동차좌표계(Homogeneous) 투영
    pixel_homo = K * X_cam;
    
    % 4. 비동차좌표계(Inhomogeneous)로 변환
    pixel = pixel_homo(1:2) / pixel_homo(3);
    
    % 5. 이미지 경계 내에 있는지 확인
    is_visible = (pixel(1) >= 0 && pixel(1) <= img_w && ...
                  pixel(2) >= 0 && pixel(2) <= img_h);
end

function vertices = GenerateCube(size, center)
    % 지정된 중심과 크기를 갖는 큐브의 8개 정점을 생성
    h = size/2;
    v = [ -h, -h, -h;  % 1
           h, -h, -h;  % 2
           h,  h, -h;  % 3
          -h,  h, -h;  % 4
          -h, -h,  h;  % 5
           h, -h,  h;  % 6
           h,  h,  h;  % 7
          -h,  h,  h]; % 8
    vertices = (v + center')';
end

function R = LookAtTarget(target, camera_pos)
    % 카메라가 특정 지점(target)을 바라보도록 하는 회전 행렬(R_C_W) 생성
    z_c = (target - camera_pos) / norm(target - camera_pos); % 카메라의 z축
    
    % 임시 up-vector (월드 좌표계의 y축)
    up_temp = [0; 0; 1]; 
    if abs(dot(z_c, up_temp)) > 0.99
        up_temp = [0; 1; 0]; % z축과 평행할 경우를 대비
    end
    
    x_c = cross(up_temp, z_c) / norm(cross(up_temp, z_c)); % 카메라의 x축
    y_c = cross(z_c, x_c); % 카메라의 y축
    
    % 월드 좌표를 카메라 좌표로 변환하는 회전 행렬
    R = [x_c'; y_c'; z_c'];
end

%% --- 시각화 헬퍼 함수 ---

function DrawWorldAxes(length, colors)
    % 월드 좌표축 (X, Y, Z) 그리기
    line([0 length], [0 0], [0 0], 'Color', colors.x_axis, 'LineWidth', 2);
    line([0 0], [0 length], [0 0], 'Color', colors.y_axis, 'LineWidth', 2);
    line([0 0], [0 0], [0 length], 'Color', colors.z_axis, 'LineWidth', 2);
    text(length, 0, 0, 'X_W');
    text(0, length, 0, 'Y_W');
    text(0, 0, length, 'Z_W');
end

function DrawCamera(R_W_C, t_W_C, scale, colors)
    % 3D 공간에 카메라 위치와 자세 그리기
    axes_cam = scale * eye(3);
    axes_world = R_W_C * axes_cam;
    
    pos = t_W_C;
    line([pos(1) pos(1)+axes_world(1,1)], [pos(2) pos(2)+axes_world(2,1)], [pos(3) pos(3)+axes_world(3,1)], 'Color', colors.x_axis, 'LineWidth', 2);
    line([pos(1) pos(1)+axes_world(1,2)], [pos(2) pos(2)+axes_world(2,2)], [pos(3) pos(3)+axes_world(3,2)], 'Color', colors.y_axis, 'LineWidth', 2);
    line([pos(1) pos(1)+axes_world(1,3)], [pos(2) pos(2)+axes_world(2,3)], [pos(3) pos(3)+axes_world(3,3)], 'Color', colors.z_axis, 'LineWidth', 2);
    
    scatter3(pos(1), pos(2), pos(3), 100, colors.camera, 'filled', 'MarkerEdgeColor', 'k');
end

function DrawCube3D(vertices, color)
    % 3D 큐브 그리기
    edges = [1 2; 2 3; 3 4; 4 1; 5 6; 6 7; 7 8; 8 5; 1 5; 2 6; 3 7; 4 8];
    for i = 1:size(edges, 1)
        plot3(vertices(1, edges(i,:)), vertices(2, edges(i,:)), vertices(3, edges(i,:)), '-', 'Color', color, 'LineWidth', 2);
    end
    scatter3(vertices(1,:), vertices(2,:), vertices(3,:), 80, color, 'filled');
end

function DrawCube2D(pixels, visible, color)
    % 2D 이미지 평면에 투영된 큐브 그리기
    edges = [1 2; 2 3; 3 4; 4 1; 5 6; 6 7; 7 8; 8 5; 1 5; 2 6; 3 7; 4 8];
    for i = 1:size(edges, 1)
        % 두 꼭짓점이 모두 보일 때만 선을 그림
        if visible(edges(i,1)) && visible(edges(i,2))
            line(pixels(1, edges(i,:)), pixels(2, edges(i,:)), 'Color', color, 'LineWidth', 2);
        end
    end
    
    vis_idx = find(visible);
    if ~isempty(vis_idx)
        scatter(pixels(1,vis_idx), pixels(2,vis_idx), 80, color, 'filled');
        for i = 1:length(vis_idx)
            text(pixels(1,vis_idx(i))+10, pixels(2,vis_idx(i)), sprintf('v%d', vis_idx(i)));
        end
    end
end
