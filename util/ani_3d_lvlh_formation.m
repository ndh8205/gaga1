function ani_3d_lvlh_formation(sensor_data, initial_pointcloud, camera_params, ...
                               LVLH_pos, euler_angles, colors, r_init_B, t1)
% ANIMATE_CAMERA_SENSOR 카메라 센서 애니메이션 함수
%
% Inputs:
%   sensor_data        - 센서 데이터 구조체 배열
%   initial_pointcloud - Chief 포인트 클라우드 (3×N)
%   camera_params      - 카메라 파라미터 구조체
%   LVLH_pos          - LVLH 위치 궤적 (3×N)
%   euler_angles      - 오일러각 시계열 (3×N) [deg]
%   colors            - 색상 정의 구조체
%   r_init_B          - GCO 형성 반경 [km]
%   t1                - 시간 벡터

if ~exist('initial_pointcloud', 'var') || isempty(initial_pointcloud) || ~isfield(sensor_data, 'z_measured')
    warning('필요한 데이터가 없어 애니메이션을 실행할 수 없습니다.');
    return;
end

fig = figure('Name', 'Integrated Sensor Model with Attitude', 'Position', [50, 50, 1900, 950]);
fprintf('통합 애니메이션 시작 (Body + Camera + Measurements)...\n');

% 애니메이션 속도 설정
skip_frame = 10;

% 센서 파라미터
fov_camera = camera_params.intrinsic.fov;
fov_length = r_init_B * 0.8;
num_points = size(initial_pointcloud, 2);

% 센서 색상
color_cam = [0.2, 0.6, 1.0];

for k = 1:skip_frame:length(t1)
    % 카메라 데이터가 없는 시점 스킵
    if isempty(sensor_data(k).z_measured)
        continue;
    end
    
    % 데이터 불러오기
    current_time = sensor_data(k).time;
    z_measured = sensor_data(k).z_measured;
    z_true = sensor_data(k).z_true;
    current_pos = sensor_data(k).LVLH_pos;
    q_L2A = sensor_data(k).q_L2A;
    q_L2B = sensor_data(k).q_L2B;
    DCM_L2B = sensor_data(k).DCM_L2B;
    euler = euler_angles(:, k);
    
    % 3D 포인트 (LVLH에서 회전된 상태)
    R_L2A = GetDCM_QUAT(q_L2A);
    points_3d_lvlh = R_L2A * initial_pointcloud;
    
    % 픽셀 좌표와 가시성 분리
    pixels_true = z_true(1:2, :);
    visible_true = z_true(3, :);
    pixels_measured = z_measured(1:2, :);
    visible_measured = z_measured(3, :);
    
    % Body to LVLH 변환
    R_B2L = DCM_L2B';
    
    % Body 축 (Body frame 정의 → LVLH 변환)
    body_x_B = [1; 0; 0];
    body_y_B = [0; 1; 0];
    body_z_B = [0; 0; 1];
    
    body_x_L = R_B2L * body_x_B;
    body_y_L = R_B2L * body_y_B;
    body_z_L = R_B2L * body_z_B;
    
    % Camera 축 (Body frame 정의 → LVLH 변환)
    cam_x_B = [1; 0; 0];  % Camera X = Body X
    cam_y_B = [0; 0; 1];  % Camera Y = Body Z
    cam_z_B = [0; 1; 0];  % Camera Z = Body Y (광축)
    
    cam_x_L = R_B2L * cam_x_B;
    cam_y_L = R_B2L * cam_y_B;
    cam_z_L = R_B2L * cam_z_B;
    
    % --- 서브플롯 1: 통합 3D LVLH 뷰 ---
    subplot(1, 2, 1);
    cla; hold on;
    
    % LVLH 축 (참조용, 얇고 연하게)
    axis_length = r_init_B * 0.4;
    quiver3(0, 0, 0, axis_length, 0, 0, 'Color', [0.8 0.8 0.8], ...
            'LineWidth', 1.5, 'MaxHeadSize', 0.3, 'LineStyle', '--');
    text(axis_length*1.1, 0, 0, 'X', 'Color', [0.6 0.6 0.6], 'FontSize', 10);
    
    quiver3(0, 0, 0, 0, axis_length, 0, 'Color', [0.8 0.8 0.8], ...
            'LineWidth', 1.5, 'MaxHeadSize', 0.3, 'LineStyle', '--');
    text(0, axis_length*1.1, 0, 'Y', 'Color', [0.6 0.6 0.6], 'FontSize', 10);
    
    quiver3(0, 0, 0, 0, 0, axis_length, 'Color', [0.8 0.8 0.8], ...
            'LineWidth', 1.5, 'MaxHeadSize', 0.3, 'LineStyle', '--');
    text(0, 0, axis_length*1.1, 'Z', 'Color', [0.6 0.6 0.6], 'FontSize', 10);
    
    % 포인트 클라우드 (Chief)
    plot3(points_3d_lvlh(1,:), points_3d_lvlh(2,:), points_3d_lvlh(3,:), ...
          '.', 'Color', colors.points, 'MarkerSize', 5);
    
    % ========== Body 프레임 (굵게) ==========
    body_axis_len = r_init_B * 0.35;
    
    quiver3(current_pos(1), current_pos(2), current_pos(3), ...
            body_x_L(1)*body_axis_len, body_x_L(2)*body_axis_len, body_x_L(3)*body_axis_len, ...
            'Color', colors.sat_body_x, 'LineWidth', 4, 'MaxHeadSize', 0.5);
    text(current_pos(1)+body_x_L(1)*body_axis_len*1.15, ...
         current_pos(2)+body_x_L(2)*body_axis_len*1.15, ...
         current_pos(3)+body_x_L(3)*body_axis_len*1.15, ...
         'Body X', 'Color', colors.sat_body_x, 'FontWeight', 'bold', 'FontSize', 11);
    
    quiver3(current_pos(1), current_pos(2), current_pos(3), ...
            body_y_L(1)*body_axis_len, body_y_L(2)*body_axis_len, body_y_L(3)*body_axis_len, ...
            'Color', colors.sat_body_y, 'LineWidth', 4, 'MaxHeadSize', 0.5);
    text(current_pos(1)+body_y_L(1)*body_axis_len*1.15, ...
         current_pos(2)+body_y_L(2)*body_axis_len*1.15, ...
         current_pos(3)+body_y_L(3)*body_axis_len*1.15, ...
         'Body Y', 'Color', colors.sat_body_y, 'FontWeight', 'bold', 'FontSize', 11);
    
    quiver3(current_pos(1), current_pos(2), current_pos(3), ...
            body_z_L(1)*body_axis_len, body_z_L(2)*body_axis_len, body_z_L(3)*body_axis_len, ...
            'Color', colors.sat_body_z, 'LineWidth', 4, 'MaxHeadSize', 0.5);
    text(current_pos(1)+body_z_L(1)*body_axis_len*1.15, ...
         current_pos(2)+body_z_L(2)*body_axis_len*1.15, ...
         current_pos(3)+body_z_L(3)*body_axis_len*1.15, ...
         'Body Z', 'Color', colors.sat_body_z, 'FontWeight', 'bold', 'FontSize', 11);
    
    % ========== Camera 좌표축 + 광축 ==========
    cam_axis_len = r_init_B * 0.28;
    
    quiver3(current_pos(1), current_pos(2), current_pos(3), ...
            cam_x_L(1)*cam_axis_len, cam_x_L(2)*cam_axis_len, cam_x_L(3)*cam_axis_len, ...
            'Color', color_cam, 'LineWidth', 2.2, 'MaxHeadSize', 0.35, 'LineStyle', ':');
    text(current_pos(1)+cam_x_L(1)*cam_axis_len*1.15, ...
         current_pos(2)+cam_x_L(2)*cam_axis_len*1.15, ...
         current_pos(3)+cam_x_L(3)*cam_axis_len*1.15, ...
         'Cam X', 'Color', color_cam, 'FontSize', 9);
    
    quiver3(current_pos(1), current_pos(2), current_pos(3), ...
            cam_y_L(1)*cam_axis_len, cam_y_L(2)*cam_axis_len, cam_y_L(3)*cam_axis_len, ...
            'Color', color_cam, 'LineWidth', 2.2, 'MaxHeadSize', 0.35, 'LineStyle', ':');
    text(current_pos(1)+cam_y_L(1)*cam_axis_len*1.15, ...
         current_pos(2)+cam_y_L(2)*cam_axis_len*1.15, ...
         current_pos(3)+cam_y_L(3)*cam_axis_len*1.15, ...
         'Cam Y', 'Color', color_cam, 'FontSize', 9);
    
    % Camera 광축 (굵게)
    quiver3(current_pos(1), current_pos(2), current_pos(3), ...
            cam_z_L(1)*fov_length, cam_z_L(2)*fov_length, cam_z_L(3)*fov_length, ...
            'Color', color_cam, 'LineWidth', 4.5, 'MaxHeadSize', 0.4);
    text(current_pos(1)+cam_z_L(1)*fov_length*0.95, ...
         current_pos(2)+cam_z_L(2)*fov_length*0.95, ...
         current_pos(3)+cam_z_L(3)*fov_length*0.95, ...
         'Camera', 'Color', color_cam, 'FontWeight', 'bold', 'FontSize', 10);
    
    % ========== Camera FOV Cone ==========
    cone_theta = linspace(0, 2*pi, 24);
    cone_radius_cam = fov_length * tan(fov_camera / 2);
    
    circle_cam = [cone_radius_cam * cos(cone_theta); 
                  cone_radius_cam * sin(cone_theta); 
                  fov_length * ones(1, length(cone_theta))];
    
    R_C2L = [cam_x_L, cam_y_L, cam_z_L];
    circle_cam_lvlh = R_C2L * circle_cam + current_pos;
    
    % Cone 표면 선
    for i = 1:4:length(cone_theta)
        plot3([current_pos(1), circle_cam_lvlh(1,i)], ...
              [current_pos(2), circle_cam_lvlh(2,i)], ...
              [current_pos(3), circle_cam_lvlh(3,i)], ...
              'Color', [color_cam 0.3], 'LineWidth', 0.6);
    end
    
    % Cone 끝단 원
    plot3(circle_cam_lvlh(1,:), circle_cam_lvlh(2,:), circle_cam_lvlh(3,:), ...
          'Color', color_cam, 'LineWidth', 2);
    
    % ========== 궤도 경로 ==========
    plot3(LVLH_pos(1,:), LVLH_pos(2,:), LVLH_pos(3,:), ':', ...
          'Color', [0.7 0.7 0.7], 'LineWidth', 0.8);
    plot3(LVLH_pos(1,1:k), LVLH_pos(2,1:k), LVLH_pos(3,1:k), ...
          'Color', colors.deputy, 'LineWidth', 2);
    
    % Deputy 현재 위치 마커
    plot3(current_pos(1), current_pos(2), current_pos(3), ...
          'mo', 'MarkerSize', 10, 'MarkerFaceColor', 'm', 'LineWidth', 2);
    
    % Chief 위치 (원점)
    plot3(0, 0, 0, 'yo', 'MarkerSize', 12, 'MarkerFaceColor', 'y', 'LineWidth', 2);
    
    hold off;
    axis equal; grid on;
    lim = r_init_B;
    xlim([-lim, lim]); ylim([-lim, lim]); zlim([-lim, lim]);
    xlabel('Radial (x) [km]', 'FontWeight', 'bold', 'FontSize', 11); 
    ylabel('In-track (y) [km]', 'FontWeight', 'bold', 'FontSize', 11); 
    zlabel('Cross-track (z) [km]', 'FontWeight', 'bold', 'FontSize', 11);
    % view(45, 25);
    view(45, 45);
    title(sprintf(['3D LVLH: Body + Camera\n' ...
                  'Roll: %.1f° | Pitch: %.1f° | Yaw: %.1f°\n' ...
                  'Visible: %d points'], ...
                  euler(1), euler(2), euler(3), sum(visible_true > 0.5)), ...
          'FontSize', 12, 'FontWeight', 'bold');
    
    % --- 서브플롯 2: 측정된 카메라 뷰 ---
    subplot(1, 2, 2);
    cla; hold on;
    
    % 카메라 프레임
    rectangle('Position', [0, 0, camera_params.intrinsic.image_width, ...
                          camera_params.intrinsic.image_height], ...
              'EdgeColor', 'k', 'LineWidth', 2.5);
    
    % 측정된 포인트
    visible_idx = find(visible_measured > 0.5);
    if ~isempty(visible_idx)
        plot(pixels_measured(1, visible_idx), pixels_measured(2, visible_idx), ...
             '.', 'Color', colors.meas_proj, 'MarkerSize', 10);
    end
    
    % 카메라 중심
    cx = camera_params.intrinsic.K_ideal(1,3);
    cy = camera_params.intrinsic.K_ideal(2,3);
    plot(cx, cy, 'k+', 'MarkerSize', 20, 'LineWidth', 3);
    
    % 십자선 (중심 표시)
    plot([cx-50, cx+50], [cy, cy], 'k--', 'LineWidth', 1);
    plot([cx, cx], [cy-50, cy+50], 'k--', 'LineWidth', 1);
    
    % 픽셀 오차 정보
    both_visible = (visible_true > 0.5) & (visible_measured > 0.5);
    if sum(both_visible) > 0
        pixel_error = pixels_measured(:, both_visible) - pixels_true(:, both_visible);
        rms_error = sqrt(mean(pixel_error(:).^2));
        mean_u_err = mean(pixel_error(1,:));
        mean_v_err = mean(pixel_error(2,:));
        
        text(20, 50, sprintf('RMS Error: %.3f px', rms_error), ...
             'FontSize', 10, 'FontWeight', 'bold', 'Color', 'r');
        text(20, 90, sprintf('Mean u: %.3f px', mean_u_err), ...
             'FontSize', 10, 'Color', 'k');
        text(20, 120, sprintf('Mean v: %.3f px', mean_v_err), ...
             'FontSize', 10, 'Color', 'k');
    end
    
    hold off;
    xlim([-50, camera_params.intrinsic.image_width+50]);
    ylim([-50, camera_params.intrinsic.image_height+50]);
    set(gca, 'YDir', 'reverse');
    axis equal; grid on;
    xlabel('u [pixels]', 'FontWeight', 'bold', 'FontSize', 11); 
    ylabel('v [pixels]', 'FontWeight', 'bold', 'FontSize', 11);
    title(sprintf(['Measured Camera View\n' ...
                  'Visible: %d / %d points\n' ...
                  'FOV: %.1f°'], ...
                 sum(visible_measured > 0.5), num_points, rad2deg(fov_camera)), ...
          'FontSize', 12, 'FontWeight', 'bold');
    
    % 전체 제목
    rotation_rate_dps = 0.3;  % Chief rotation rate
    sgtitle(sprintf(['Integrated Sensor Model: Body + Camera\n' ...
                    't = %.1f s | Chief Rotation: %.1f deg/s'], ...
                    current_time, rotation_rate_dps), ...
            'FontSize', 15, 'FontWeight', 'bold');
    
    drawnow;
    
    % 센서 상태 출력 (주기적으로)
    if mod(k, skip_frame*5) == 1 || k == 1
        fprintf('t=%.1f s: ', current_time);
        
        % 카메라 픽셀 오차
        if sum(both_visible) > 0
            fprintf('RMS: %.3f px | ', rms_error);
        end
        
        fprintf('Visible: %d/%d | Euler: [%.1f, %.1f, %.1f]°\n', ...
                sum(visible_measured > 0.5), num_points, ...
                euler(1), euler(2), euler(3));
    end
end

fprintf('통합 애니메이션 완료!\n');

end