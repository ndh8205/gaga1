function ani_3d_lvlh_formation_v3_gaim(sensor_data, initial_pointcloud, camera_params, ...
                                   LVLH_pos, euler_angles, colors, r_init_B, t1)
% V3: 완전한 3D 모델 투영 (vertices + faces) 지원
%
% Inputs:
%   sensor_data - 센서 데이터 구조체 배열
%   initial_pointcloud - 3xN 포인트클라우드 또는 model_data 구조체
%   camera_params - 카메라 파라미터
%   LVLH_pos - LVLH 위치 시계열
%   euler_angles - 오일러 각 시계열
%   colors - 색상 구조체
%   r_init_B - 초기 상대 거리
%   t1 - 시간 벡터

% ---------- 초기 설정 ----------
if ~exist('initial_pointcloud', 'var') || isempty(initial_pointcloud) || ~isfield(sensor_data, 'z_measured')
    warning('필요한 데이터가 없어 애니메이션을 실행할 수 없습니다.');
    return;
end

% Figure 생성
fig = figure('Name', 'LVLH Formation V3 - Full 3D Model Projection', ...
            'Position', [50, 50, 1900, 950], 'Color', 'k');
fprintf('통합 애니메이션 V3 시작 (완전한 3D 모델 투영)...\n');

% 애니메이션 파라미터
skip_frame = 10;  % 프레임 스킵
fov_camera = camera_params.intrinsic.fov;
fov_length = r_init_B * 0.8;
K = camera_params.intrinsic.K_ideal;

% 색상 설정
color_cam = [0.2, 0.6, 1.0];
color_chief_face = [0.7, 0.7, 0.8];
color_chief_edge = [0.9, 0.9, 0.95];
color_deputy_body = [0.8, 0.3, 0.3];

% 모델 데이터 체크
if isstruct(initial_pointcloud)
    model_vertices = initial_pointcloud.vertices;
    model_faces = initial_pointcloud.faces;
    has_faces = ~isempty(model_faces);
else
    model_vertices = initial_pointcloud;
    model_faces = [];
    has_faces = false;
end
num_points = size(model_vertices, 2);

% 진행 상태 표시
total_frames = floor(length(t1) / skip_frame);
frame_count = 0;

% ---------- 메인 애니메이션 루프 ----------
for k = 1:skip_frame:length(t1)
    if isempty(sensor_data(k).z_measured)
        continue;
    end
    
    frame_count = frame_count + 1;
    
    % 현재 상태 추출
    current_time = sensor_data(k).time;
    z_measured = sensor_data(k).z_measured;
    z_true = sensor_data(k).z_true;
    current_pos = sensor_data(k).LVLH_pos;
    q_L2A = sensor_data(k).q_L2A;
    q_L2B = sensor_data(k).q_L2B;
    euler = euler_angles(:, k);
    
    % 좌표 변환
    R_L2A = GetDCM_QUAT(q_L2A);
    points_3d_lvlh = R_L2A * model_vertices;
    
    % 측정값 분해
    if isstruct(z_true)
        pixels_true = z_true.pixels;
        visible_true = z_true.visible;
        if isfield(z_true, 'face_info')
            face_info_true = z_true.face_info;
            has_face_data = true;
        else
            has_face_data = false;
        end
    else
        pixels_true = z_true(1:2, :);
        visible_true = z_true(3, :);
        has_face_data = false;
    end
    
    if isstruct(z_measured)
        pixels_measured = z_measured.pixels;
        visible_measured = z_measured.visible;
    else
        pixels_measured = z_measured(1:2, :);
        visible_measured = z_measured(3, :);
    end
    
    % Body/Camera 축 계산
    DCM_L2B = GetDCM_QUAT(q_L2B);
    R_B2L = DCM_L2B';
    
    body_x_L = R_B2L * [1; 0; 0];
    body_y_L = R_B2L * [0; 1; 0];
    body_z_L = R_B2L * [0; 0; 1];
    
    cam_x_L = R_B2L * [1; 0; 0];
    cam_y_L = R_B2L * [0; 0; 1];
    cam_z_L = R_B2L * [0; 1; 0];
    
    % ========== 서브플롯 1: 3D LVLH 뷰 ==========
    subplot(2, 3, [1 2 4 5]);  % 왼쪽 2/3 공간 사용
    cla; hold on;
    set(gca, 'Color', [0.05 0.05 0.1]);  % 진한 남색 배경
    
    % LVLH 기준 축
    axis_length = r_init_B * 0.4;
    quiver3(0, 0, 0, axis_length, 0, 0, 'Color', [0.6 0.6 0.6], 'LineWidth', 1, 'LineStyle', ':');
    quiver3(0, 0, 0, 0, axis_length, 0, 'Color', [0.6 0.6 0.6], 'LineWidth', 1, 'LineStyle', ':');
    quiver3(0, 0, 0, 0, 0, axis_length, 'Color', [0.6 0.6 0.6], 'LineWidth', 1, 'LineStyle', ':');
    text(axis_length*1.1, 0, 0, 'X', 'Color', [0.8 0.8 0.8], 'FontSize', 9);
    text(0, axis_length*1.1, 0, 'Y', 'Color', [0.8 0.8 0.8], 'FontSize', 9);
    text(0, 0, axis_length*1.1, 'Z', 'Color', [0.8 0.8 0.8], 'FontSize', 9);
    
    % Chief 3D 모델 (faces가 있으면 faces로, 없으면 points로)
    if has_faces && ~isempty(model_faces)
        % Face 렌더링
        patch('Vertices', points_3d_lvlh', ...
              'Faces', model_faces, ...
              'FaceColor', color_chief_face, ...
              'EdgeColor', color_chief_edge, ...
              'FaceAlpha', 0.9, ...
              'EdgeAlpha', 0.3, ...
              'LineWidth', 0.1);
    else
        % Point cloud 렌더링
        scatter3(points_3d_lvlh(1,:), points_3d_lvlh(2,:), points_3d_lvlh(3,:), ...
                2, colors.points, 'filled');
    end
    
    % Deputy Body 축
    body_axis_len = r_init_B * 0.3;
    quiver3(current_pos(1), current_pos(2), current_pos(3), ...
            body_x_L(1)*body_axis_len, body_x_L(2)*body_axis_len, body_x_L(3)*body_axis_len, ...
            'Color', colors.sat_body_x, 'LineWidth', 3, 'MaxHeadSize', 0.5);
    quiver3(current_pos(1), current_pos(2), current_pos(3), ...
            body_y_L(1)*body_axis_len, body_y_L(2)*body_axis_len, body_y_L(3)*body_axis_len, ...
            'Color', colors.sat_body_y, 'LineWidth', 3, 'MaxHeadSize', 0.5);
    quiver3(current_pos(1), current_pos(2), current_pos(3), ...
            body_z_L(1)*body_axis_len, body_z_L(2)*body_axis_len, body_z_L(3)*body_axis_len, ...
            'Color', colors.sat_body_z, 'LineWidth', 3, 'MaxHeadSize', 0.5);
    
    % Camera 시야 (FOV cone)
    quiver3(current_pos(1), current_pos(2), current_pos(3), ...
            cam_z_L(1)*fov_length, cam_z_L(2)*fov_length, cam_z_L(3)*fov_length, ...
            'Color', color_cam, 'LineWidth', 3, 'MaxHeadSize', 0.3);
    
    % FOV Cone 원
    cone_theta = linspace(0, 2*pi, 32);
    cone_radius_cam = fov_length * tan(fov_camera / 2);
    circle_cam = [cone_radius_cam * cos(cone_theta); 
                  cone_radius_cam * sin(cone_theta); 
                  fov_length * ones(1, length(cone_theta))];
    R_C2L = [cam_x_L, cam_y_L, cam_z_L];
    circle_cam_lvlh = R_C2L * circle_cam + current_pos;
    
    % FOV 원뿔 선
    plot3(circle_cam_lvlh(1,:), circle_cam_lvlh(2,:), circle_cam_lvlh(3,:), ...
          'Color', color_cam, 'LineWidth', 1.5);
    for i = 1:8:length(cone_theta)
        plot3([current_pos(1), circle_cam_lvlh(1,i)], ...
              [current_pos(2), circle_cam_lvlh(2,i)], ...
              [current_pos(3), circle_cam_lvlh(3,i)], ...
              'Color', color_cam, 'LineWidth', 0.5, 'LineStyle', '--');
    end
    
    % 궤적
    plot3(LVLH_pos(1,1:k), LVLH_pos(2,1:k), LVLH_pos(3,1:k), ...
          'Color', [0.8, 0.4, 0.4], 'LineWidth', 1.5);
    
    % Deputy 위치
    plot3(current_pos(1), current_pos(2), current_pos(3), ...
          'o', 'Color', color_deputy_body, 'MarkerSize', 10, ...
          'MarkerFaceColor', color_deputy_body);
    
    % Chief 위치
    plot3(0, 0, 0, 's', 'Color', colors.chief, 'MarkerSize', 12, ...
          'MarkerFaceColor', colors.chief);
    
    hold off;
    axis equal; grid on;
    lim = r_init_B * 1.2;
    xlim([-lim, lim]); ylim([-lim, lim]); zlim([-lim, lim]);
    xlabel('Radial [km]', 'Color', 'w', 'FontSize', 10); 
    ylabel('In-track [km]', 'Color', 'w', 'FontSize', 10); 
    zlabel('Cross-track [km]', 'Color', 'w', 'FontSize', 10);
    set(gca, 'XColor', 'w', 'YColor', 'w', 'ZColor', 'w', 'GridColor', [0.3 0.3 0.3]);
    view(-45, 25);
    title(sprintf('3D LVLH View (Distance: %.3f km)', norm(current_pos)), ...
          'Color', 'w', 'FontSize', 11);
    
    % ========== 서브플롯 2: 카메라 픽셀 뷰 ==========
    subplot(2, 3, [3 6]);  % 오른쪽 1/3 공간 사용
    cla; hold on;
    
    % 지구 배경 또는 검은 배경
    if isfield(sensor_data(k), 'earth_image') && ~isempty(sensor_data(k).earth_image)
        imshow(sensor_data(k).earth_image);
    else
        set(gca, 'Color', 'k');
        % 격자 패턴 (옵션)
        [X, Y] = meshgrid(0:100:camera_params.intrinsic.image_width, ...
                         0:100:camera_params.intrinsic.image_height);
        plot(X, Y, 'Color', [0.1 0.1 0.1], 'LineWidth', 0.5);
        plot(X', Y', 'Color', [0.1 0.1 0.1], 'LineWidth', 0.5);
    end
    
    % 카메라 프레임
    rectangle('Position', [0, 0, camera_params.intrinsic.image_width, ...
              camera_params.intrinsic.image_height], ...
              'EdgeColor', 'y', 'LineWidth', 2);
    
    % Face 또는 Point 렌더링
    if has_face_data && has_faces
        % Faces 렌더링
        face_colors = repmat(color_chief_face, size(face_info_true.faces, 1), 1);
        
        % Depth에 따른 명암 조절
        if ~isempty(face_info_true.depths)
            depth_norm = (face_info_true.depths - min(face_info_true.depths)) / ...
                        (max(face_info_true.depths) - min(face_info_true.depths) + eps);
            brightness = 0.5 + 0.5 * (1 - depth_norm);
        else
            brightness = ones(size(face_info_true.faces, 1), 1);
        end
        
        % 가시적인 faces만 그리기
        for i = 1:length(face_info_true.depth_order)
            f = face_info_true.depth_order(i);
            if face_info_true.visible(f)
                idx = face_info_true.faces(f, :);
                
                % 유효한 인덱스 체크
                if all(idx <= num_points) && all(visible_true(idx))
                    tri_x = pixels_true(1, idx([1 2 3 1]));
                    tri_y = pixels_true(2, idx([1 2 3 1]));
                    
                    % Face 색상 (depth에 따른 명암)
                    face_color = color_chief_face * brightness(f);
                    
                    % Filled triangle
                    fill(tri_x(1:3), tri_y(1:3), face_color, ...
                         'EdgeColor', color_chief_edge, ...
                         'LineWidth', 0.3, ...
                         'FaceAlpha', 0.9);
                end
            end
        end
        visible_str = sprintf('Visible Faces: %d/%d', ...
                             sum(face_info_true.visible), ...
                             length(face_info_true.visible));
    else
        % Points 렌더링
        visible_idx = find(visible_measured > 0.5);
        if ~isempty(visible_idx)
            scatter(pixels_measured(1, visible_idx), ...
                   pixels_measured(2, visible_idx), ...
                   5, colors.meas_proj, 'filled');
        end
        visible_str = sprintf('Visible Points: %d/%d', ...
                             sum(visible_measured > 0.5), num_points);
    end
    
    % 광학 중심점
    cx = K(1,3); cy = K(2,3);
    plot(cx, cy, 'r+', 'MarkerSize', 15, 'LineWidth', 2);
    
    % FOV 표시
    fov_deg = rad2deg(fov_camera);
    text(50, 50, sprintf('FOV: %.1f°', fov_deg), ...
         'Color', 'y', 'FontSize', 10);
    
    hold off;
    xlim([0, camera_params.intrinsic.image_width]);
    ylim([0, camera_params.intrinsic.image_height]);
    set(gca, 'YDir', 'reverse', 'XColor', 'w', 'YColor', 'w');
    axis equal; grid off;
    
    xlabel('u [pixels]', 'Color', 'w', 'FontSize', 10); 
    ylabel('v [pixels]', 'Color', 'w', 'FontSize', 10);
    title(sprintf('Camera View\n%s', visible_str), ...
          'Color', 'w', 'FontSize', 11);
    
    % ========== 전체 제목 ==========
    sgtitle(sprintf('Time: %.1f s | Frame: %d/%d | Roll: %.1f° Pitch: %.1f° Yaw: %.1f°', ...
            current_time, frame_count, total_frames, ...
            euler(1), euler(2), euler(3)), ...
            'FontSize', 14, 'FontWeight', 'bold', 'Color', 'w');
    
    drawnow;
    
    % 진행 상황 출력 (10 프레임마다)
    if mod(frame_count, 10) == 0
        fprintf('Frame %d/%d: t=%.1f s, Distance=%.3f km, %s\n', ...
                frame_count, total_frames, current_time, ...
                norm(current_pos), visible_str);
    end
end

fprintf('애니메이션 V3 완료! (총 %d 프레임)\n', frame_count);
end