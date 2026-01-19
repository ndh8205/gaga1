function ani_3d_lvlh_formation_v2(sensor_data, initial_pointcloud, camera_params, ...
                                   LVLH_pos, euler_angles, colors, r_init_B, t1)
% V2: 카메라 픽셀 뷰에 오프스크린 렌더링 지구 표시

if ~exist('initial_pointcloud', 'var') || isempty(initial_pointcloud) || ~isfield(sensor_data, 'z_measured')
    warning('필요한 데이터가 없어 애니메이션을 실행할 수 없습니다.');
    return;
end

fig = figure('Name', 'Integrated Sensor Model with Earth', 'Position', [50, 50, 1900, 950], 'Color', 'k');
fprintf('통합 애니메이션 v2 시작 (오프스크린 렌더링 지구 표시)...\n');

skip_frame = 10;
fov_camera = camera_params.intrinsic.fov;
fov_length = r_init_B * 0.8;
num_points = size(initial_pointcloud, 2);
K = camera_params.intrinsic.K_ideal;

color_cam = [0.2, 0.6, 1.0];

for k = 1:skip_frame:length(t1)
    if isempty(sensor_data(k).z_measured)
        continue;
    end
    
    current_time = sensor_data(k).time;
    z_measured = sensor_data(k).z_measured;
    z_true = sensor_data(k).z_true;
    current_pos = sensor_data(k).LVLH_pos;
    q_L2A = sensor_data(k).q_L2A;
    q_L2B = sensor_data(k).q_L2B;
    euler = euler_angles(:, k);
    
    R_L2A = GetDCM_QUAT(q_L2A);
    points_3d_lvlh = R_L2A * initial_pointcloud;
    
    pixels_true = z_true(1:2, :);
    visible_true = z_true(3, :);
    pixels_measured = z_measured(1:2, :);
    visible_measured = z_measured(3, :);
    
    DCM_L2B = GetDCM_QUAT(q_L2B);
    R_B2L = DCM_L2B';
    
    % Body/Camera 축
    body_x_L = R_B2L * [1; 0; 0];
    body_y_L = R_B2L * [0; 1; 0];
    body_z_L = R_B2L * [0; 0; 1];
    
    cam_x_L = R_B2L * [1; 0; 0];
    cam_y_L = R_B2L * [0; 0; 1];
    cam_z_L = R_B2L * [0; 1; 0];
    
    % --- 서브플롯 1: 3D LVLH 뷰 ---
    subplot(1, 2, 1);
    cla; hold on;
    set(gca, 'Color', 'k');
    
    % LVLH 축
    axis_length = r_init_B * 0.4;
    quiver3(0, 0, 0, axis_length, 0, 0, 'Color', [0.8 0.8 0.8], 'LineWidth', 1.5, 'LineStyle', '--');
    text(axis_length*1.1, 0, 0, 'X', 'Color', [0.8 0.8 0.8], 'FontSize', 10);
    quiver3(0, 0, 0, 0, axis_length, 0, 'Color', [0.8 0.8 0.8], 'LineWidth', 1.5, 'LineStyle', '--');
    text(0, axis_length*1.1, 0, 'Y', 'Color', [0.8 0.8 0.8], 'FontSize', 10);
    quiver3(0, 0, 0, 0, 0, axis_length, 'Color', [0.8 0.8 0.8], 'LineWidth', 1.5, 'LineStyle', '--');
    text(0, 0, axis_length*1.1, 'Z', 'Color', [0.8 0.8 0.8], 'FontSize', 10);
    
    % Chief 포인트 클라우드
    plot3(points_3d_lvlh(1,:), points_3d_lvlh(2,:), points_3d_lvlh(3,:), '.', 'Color', colors.points, 'MarkerSize', 5);
    
    % Body 축
    body_axis_len = r_init_B * 0.35;
    quiver3(current_pos(1), current_pos(2), current_pos(3), ...
            body_x_L(1)*body_axis_len, body_x_L(2)*body_axis_len, body_x_L(3)*body_axis_len, ...
            'Color', colors.sat_body_x, 'LineWidth', 4, 'MaxHeadSize', 0.5);
    quiver3(current_pos(1), current_pos(2), current_pos(3), ...
            body_y_L(1)*body_axis_len, body_y_L(2)*body_axis_len, body_y_L(3)*body_axis_len, ...
            'Color', colors.sat_body_y, 'LineWidth', 4, 'MaxHeadSize', 0.5);
    quiver3(current_pos(1), current_pos(2), current_pos(3), ...
            body_z_L(1)*body_axis_len, body_z_L(2)*body_axis_len, body_z_L(3)*body_axis_len, ...
            'Color', colors.sat_body_z, 'LineWidth', 4, 'MaxHeadSize', 0.5);
    
    % Camera 광축
    quiver3(current_pos(1), current_pos(2), current_pos(3), ...
            cam_z_L(1)*fov_length, cam_z_L(2)*fov_length, cam_z_L(3)*fov_length, ...
            'Color', color_cam, 'LineWidth', 4.5, 'MaxHeadSize', 0.4);
    
    % FOV Cone
    cone_theta = linspace(0, 2*pi, 24);
    cone_radius_cam = fov_length * tan(fov_camera / 2);
    circle_cam = [cone_radius_cam * cos(cone_theta); cone_radius_cam * sin(cone_theta); fov_length * ones(1, length(cone_theta))];
    R_C2L = [cam_x_L, cam_y_L, cam_z_L];
    circle_cam_lvlh = R_C2L * circle_cam + current_pos;
    plot3(circle_cam_lvlh(1,:), circle_cam_lvlh(2,:), circle_cam_lvlh(3,:), 'Color', color_cam, 'LineWidth', 2);
    
    % 궤도
    plot3(LVLH_pos(1,1:k), LVLH_pos(2,1:k), LVLH_pos(3,1:k), 'Color', colors.deputy, 'LineWidth', 2);
    plot3(current_pos(1), current_pos(2), current_pos(3), 'mo', 'MarkerSize', 10, 'MarkerFaceColor', 'm');
    % plot3(0, 0, 0, 'yo', 'MarkerSize', 12, 'MarkerFaceColor', 'y');
    
    hold off;
    axis equal; grid on;
    lim = r_init_B;
    xlim([-lim, lim]); ylim([-lim, lim]); zlim([-lim, lim]);
    xlabel('Radial (x) [km]', 'Color', 'w'); 
    ylabel('In-track (y) [km]', 'Color', 'w'); 
    zlabel('Cross-track (z) [km]', 'Color', 'w');
    set(gca, 'XColor', 'w', 'YColor', 'w', 'ZColor', 'w');
    view(-45, 25);
    title(sprintf('3D LVLH: Visible %d pts', sum(visible_true > 0.5)), 'Color', 'w');
    
    % --- 서브플롯 2: 카메라 뷰 + 오프스크린 렌더링 지구 ---
    subplot(1, 2, 2);
    cla;
    
    if isfield(sensor_data(k), 'earth_image') && ~isempty(sensor_data(k).earth_image)
        imshow(sensor_data(k).earth_image);
        hold on;
        
        visible_idx = find(visible_measured > 0.5);
        if ~isempty(visible_idx)
            plot(pixels_measured(1, visible_idx), pixels_measured(2, visible_idx), '.', 'Color', colors.meas_proj, 'MarkerSize', 1);
        end
        
        % cx = K(1,3); cy = K(2,3);
        % plot(cx, cy, 'w+', 'MarkerSize', 20, 'LineWidth', 1);
        
        rectangle('Position', [0, 0, camera_params.intrinsic.image_width, ...
                  camera_params.intrinsic.image_height], ...
                  'EdgeColor', 'w', 'LineWidth', 2.5);
        
        hold off;
        xlim([0, camera_params.intrinsic.image_width]);
        ylim([0, camera_params.intrinsic.image_height]);
        set(gca, 'YDir', 'reverse');
    else
        hold on;
        set(gca, 'Color', 'k');
        
        rectangle('Position', [0, 0, camera_params.intrinsic.image_width, ...
                  camera_params.intrinsic.image_height], ...
                  'EdgeColor', 'w', 'LineWidth', 2.5);
        
        visible_idx = find(visible_measured > 0.5);
        if ~isempty(visible_idx)
            plot(pixels_measured(1, visible_idx), pixels_measured(2, visible_idx), ...
                 '.', 'Color', colors.meas_proj, 'MarkerSize', 10);
        end
        
        cx = K(1,3); cy = K(2,3);
        plot(cx, cy, 'w+', 'MarkerSize', 20, 'LineWidth', 3);
        
        hold off;
        xlim([-50, camera_params.intrinsic.image_width+50]);
        ylim([-50, camera_params.intrinsic.image_height+50]);
        set(gca, 'YDir', 'reverse', 'XColor', 'w', 'YColor', 'w');
    end
    
    axis equal; grid on;
    xlabel('u [pixels]', 'Color', 'w'); 
    ylabel('v [pixels]', 'Color', 'w');
    title(sprintf('Camera View (Earth Rendered)\nVisible: %d pts', sum(visible_measured > 0.5)), 'Color', 'w');
    
    sgtitle(sprintf('t = %.1f s', current_time), 'FontSize', 15, 'FontWeight', 'bold', 'Color', 'w');
    drawnow;
    
    if mod(k, skip_frame*5) == 1
        fprintf('t=%.1f s: Visible %d/%d\n', current_time, sum(visible_measured > 0.5), num_points);
    end
end

fprintf('애니메이션 v2 완료!\n');
end

