function ani_combined_lvlh_attitude(sensor_data, initial_pointcloud, camera_params, ...
                                    LVLH_pos, euler_angles, colors, r_init_B, t1, orbit_normal_vec)
% 통합 애니메이션: 3D LVLH 뷰 + Deputy 자세
%
% 왼쪽: 3D LVLH 뷰 (원본 그대로)
% 오른쪽: Deputy 자세 (Body/Camera/Star Tracker 축)

if ~exist('initial_pointcloud', 'var') || isempty(initial_pointcloud) || ~isfield(sensor_data, 'z_measured')
    warning('필요한 데이터가 없어 애니메이션을 실행할 수 없습니다.');
    return;
end

fig = figure('Name', 'Integrated Sensor Model', 'Position', [50, 50, 1800, 800]);
fprintf('애니메이션 시작 (3D LVLH + 자세)...\n');

% 애니메이션 속도 설정
skip_frame = 10;
num_points = size(initial_pointcloud, 2);

% 센서 색상
color_cam = [0.2, 0.6, 1.0];
color_st = [0.8, 0.2, 0.8];
fov_camera = camera_params.intrinsic.fov;
fov_st = deg2rad(15);
fov_length = r_init_B * 0.8;

rotation_rate_dps = 0.3;

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
    
    % 카메라 DCM 계산
    [R_L2C, R_B2L] = GetCameraFromQuat(q_L2B, current_pos, orbit_normal_vec);
    R_C2L = R_L2C';
    cam_x = R_C2L(:, 1);
    cam_y = R_C2L(:, 2);
    cam_z = R_C2L(:, 3);
    
    % Star Tracker 축
    st_x = cam_x;
    st_z = -cam_z;
    st_y = cross(st_z, st_x);
    st_y = st_y / norm(st_y);
    
    % --- 서브플롯 1: 3D LVLH 뷰 (원본 그대로) ---
    subplot(1, 4, 1);
    cla; hold on;
    
    % LVLH 축
    DrawLVLHAxes(r_init_B * 0.5, colors);
    
    % 포인트 클라우드
    DrawPoints3D(points_3d_lvlh, colors.points);
    
    % Deputy 위성 본체
    DrawSatelliteBody(R_B2L, current_pos, r_init_B * 0.2, colors);
    
    % Camera FOV Cone
    cone_theta = linspace(0, 2*pi, 20);
    cone_radius_cam = fov_length * tan(fov_camera / 2);
    circle_cam = [cone_radius_cam * cos(cone_theta); 
                  cone_radius_cam * sin(cone_theta); 
                  fov_length * ones(1, length(cone_theta))];
    circle_cam_lvlh = R_C2L * circle_cam + current_pos;
    
    for i = 1:4:length(cone_theta)
        plot3([current_pos(1), circle_cam_lvlh(1,i)], ...
              [current_pos(2), circle_cam_lvlh(2,i)], ...
              [current_pos(3), circle_cam_lvlh(3,i)], ...
              'Color', [color_cam 0.25], 'LineWidth', 0.5);
    end
    plot3(circle_cam_lvlh(1,:), circle_cam_lvlh(2,:), circle_cam_lvlh(3,:), ...
          'Color', color_cam, 'LineWidth', 1.5);
    
    % Camera 광축
    quiver3(current_pos(1), current_pos(2), current_pos(3), ...
            cam_z(1)*fov_length, cam_z(2)*fov_length, cam_z(3)*fov_length, ...
            'Color', color_cam, 'LineWidth', 3.5, 'MaxHeadSize', 0.4);
    
    % Star Tracker FOV Cone
    cone_radius_st = fov_length * tan(fov_st / 2);
    circle_st_local = [cone_radius_st * cos(cone_theta); 
                       cone_radius_st * sin(cone_theta); 
                       fov_length * ones(1, length(cone_theta))];
    R_ST2L = [st_x, st_y, st_z];
    circle_st_lvlh = R_ST2L * circle_st_local + current_pos;
    
    for i = 1:4:length(cone_theta)
        plot3([current_pos(1), circle_st_lvlh(1,i)], ...
              [current_pos(2), circle_st_lvlh(2,i)], ...
              [current_pos(3), circle_st_lvlh(3,i)], ...
              'Color', [color_st 0.25], 'LineWidth', 0.5);
    end
    plot3(circle_st_lvlh(1,:), circle_st_lvlh(2,:), circle_st_lvlh(3,:), ...
          'Color', color_st, 'LineWidth', 1.5);
    
    % Star Tracker 광축
    quiver3(current_pos(1), current_pos(2), current_pos(3), ...
            st_z(1)*fov_length, st_z(2)*fov_length, st_z(3)*fov_length, ...
            'Color', color_st, 'LineWidth', 3.5, 'MaxHeadSize', 0.4);
    
    % 궤도 경로
    plot3(LVLH_pos(1,:), LVLH_pos(2,:), LVLH_pos(3,:), ':', ...
          'Color', [0.7 0.7 0.7], 'LineWidth', 0.5);
    plot3(LVLH_pos(1,1:k), LVLH_pos(2,1:k), LVLH_pos(3,1:k), ...
          'Color', colors.deputy, 'LineWidth', 2);
    
    hold off;
    axis equal; grid on;
    
    % 3인칭 카메라 뷰 (Deputy Body Y축 뒤쪽에서 추적)
    lim = r_init_B;
    xlim([-lim, lim]); ylim([-lim, lim]); zlim([-lim, lim]);
    
    % Deputy Body 축 추출 (LVLH 좌표계에서 표현)
    body_x = R_B2L(:, 1);
    body_y = R_B2L(:, 2);
    body_z = R_B2L(:, 3);
    
    % 카메라 위치: Body Y축 뒤쪽 + Body Z축 위쪽
    camera_distance_back = lim * 1.5;
    camera_distance_up = lim * 0.5;
    
    camera_pos = current_pos - body_y * camera_distance_back + body_z * camera_distance_up;
    
    campos(camera_pos);
    camtarget(current_pos);
    camup(body_z);
    camva(45);
    
    xlabel('Radial (x) [km]'); ylabel('In-track (y) [km]'); zlabel('Cross-track (z) [km]');
    title(sprintf('3D Body Frame View - Third Person (Points: %d)', num_points));
    
    % --- 서브플롯 2: Deputy 자세 애니메이션 (2번 코드) ---
    subplot(1, 4, [3,4]);
    cla; hold on;
    
    % 자세 애니메이션 파라미터
    axis_length = 0.85;
    fov_length_att = 1.3;
    
    % LVLH 프레임 (고정, 회색, 얇게)
    quiver3(0, 0, 0, axis_length*0.6, 0, 0, 'Color', [0.75 0.75 0.75], ...
            'LineWidth', 1.5, 'MaxHeadSize', 0.4, 'LineStyle', '--');
    text(axis_length*0.7, 0, 0, 'LVLH X', 'Color', [0.5 0.5 0.5], 'FontSize', 9);
    
    quiver3(0, 0, 0, 0, axis_length*0.6, 0, 'Color', [0.75 0.75 0.75], ...
            'LineWidth', 1.5, 'MaxHeadSize', 0.4, 'LineStyle', '--');
    text(0, axis_length*0.7, 0, 'LVLH Y', 'Color', [0.5 0.5 0.5], 'FontSize', 9);
    
    quiver3(0, 0, 0, 0, 0, axis_length*0.6, 'Color', [0.75 0.75 0.75], ...
            'LineWidth', 1.5, 'MaxHeadSize', 0.4, 'LineStyle', '--');
    text(0, 0, axis_length*0.7, 'LVLH Z', 'Color', [0.5 0.5 0.5], 'FontSize', 9);
    
    % Body 프레임 in LVLH (회전, 컬러, 굵게)
    body_x = R_B2L(:, 1) * axis_length;
    body_y = R_B2L(:, 2) * axis_length;
    body_z = R_B2L(:, 3) * axis_length;
    
    quiver3(0, 0, 0, body_x(1), body_x(2), body_x(3), ...
            'Color', colors.sat_body_x, 'LineWidth', 3.5, 'MaxHeadSize', 0.5);
    text(body_x(1)*1.15, body_x(2)*1.15, body_x(3)*1.15, 'Body X', ...
         'Color', colors.sat_body_x, 'FontWeight', 'bold', 'FontSize', 11);
    
    quiver3(0, 0, 0, body_y(1), body_y(2), body_y(3), ...
            'Color', colors.sat_body_y, 'LineWidth', 3.5, 'MaxHeadSize', 0.5);
    text(body_y(1)*1.15, body_y(2)*1.15, body_y(3)*1.15, 'Body Y', ...
         'Color', colors.sat_body_y, 'FontWeight', 'bold', 'FontSize', 11);
    
    quiver3(0, 0, 0, body_z(1), body_z(2), body_z(3), ...
            'Color', colors.sat_body_z, 'LineWidth', 3.5, 'MaxHeadSize', 0.5);
    text(body_z(1)*1.15, body_z(2)*1.15, body_z(3)*1.15, 'Body Z', ...
         'Color', colors.sat_body_z, 'FontWeight', 'bold', 'FontSize', 11);
    
    % Camera 프레임 (얇게, 점선)
    cam_axis_length = axis_length * 0.7;
    
    quiver3(0, 0, 0, cam_x(1)*cam_axis_length, cam_x(2)*cam_axis_length, ...
            cam_x(3)*cam_axis_length, 'Color', [1, 0.5, 0.5], ...
            'LineWidth', 2, 'MaxHeadSize', 0.4, 'LineStyle', ':');
    text(cam_x(1)*cam_axis_length*1.2, cam_x(2)*cam_axis_length*1.2, ...
         cam_x(3)*cam_axis_length*1.2, 'Cam X', ...
         'Color', [0.8, 0, 0], 'FontSize', 9);
    
    quiver3(0, 0, 0, cam_y(1)*cam_axis_length, cam_y(2)*cam_axis_length, ...
            cam_y(3)*cam_axis_length, 'Color', [0.5, 1, 0.5], ...
            'LineWidth', 2, 'MaxHeadSize', 0.4, 'LineStyle', ':');
    text(cam_y(1)*cam_axis_length*1.2, cam_y(2)*cam_axis_length*1.2, ...
         cam_y(3)*cam_axis_length*1.2, 'Cam Y', ...
         'Color', [0, 0.6, 0], 'FontSize', 9);
    
    quiver3(0, 0, 0, cam_z(1)*fov_length_att, cam_z(2)*fov_length_att, ...
            cam_z(3)*fov_length_att, 'Color', [0.3, 0.3, 1], ...
            'LineWidth', 4, 'MaxHeadSize', 0.5);
    text(cam_z(1)*fov_length_att*1.05, cam_z(2)*fov_length_att*1.05, ...
         cam_z(3)*fov_length_att*1.05, 'Camera Axis', ...
         'Color', [0, 0, 0.8], 'FontWeight', 'bold', 'FontSize', 10);
    
    % Camera FOV Cone (자세 뷰)
    cone_radius_cam_att = fov_length_att * tan(fov_camera / 2);
    circle_cam_att = [cone_radius_cam_att * cos(cone_theta); 
                      cone_radius_cam_att * sin(cone_theta); 
                      fov_length_att * ones(1, length(cone_theta))];
    circle_cam_lvlh_att = R_C2L * circle_cam_att;
    
    for i = 1:4:length(cone_theta)
        plot3([0, circle_cam_lvlh_att(1,i)], [0, circle_cam_lvlh_att(2,i)], ...
              [0, circle_cam_lvlh_att(3,i)], 'Color', [0.6 0.6 1], 'LineWidth', 0.5);
    end
    plot3(circle_cam_lvlh_att(1,:), circle_cam_lvlh_att(2,:), circle_cam_lvlh_att(3,:), ...
          'Color', [0.2, 0.2, 1], 'LineWidth', 2);
    
    % Star Tracker 프레임과 FOV
    st_axis_length = axis_length * 0.7;
    
    quiver3(0, 0, 0, st_x(1)*st_axis_length, st_x(2)*st_axis_length, ...
            st_x(3)*st_axis_length, 'Color', [1, 0.5, 1], ...
            'LineWidth', 2, 'MaxHeadSize', 0.4, 'LineStyle', ':');
    text(st_x(1)*st_axis_length*1.2, st_x(2)*st_axis_length*1.2, ...
         st_x(3)*st_axis_length*1.2, 'ST X', ...
         'Color', color_st, 'FontSize', 9);  % color_st 사용
    
    quiver3(0, 0, 0, st_y(1)*st_axis_length, st_y(2)*st_axis_length, ...
            st_y(3)*st_axis_length, 'Color', [1, 0.5, 1], ...
            'LineWidth', 2, 'MaxHeadSize', 0.4, 'LineStyle', ':');
    text(st_y(1)*st_axis_length*1.2, st_y(2)*st_axis_length*1.2, ...
         st_y(3)*st_axis_length*1.2, 'ST Y', ...
         'Color', color_st, 'FontSize', 9);  % color_st 사용
    
    quiver3(0, 0, 0, st_z(1)*fov_length_att, st_z(2)*fov_length_att, ...
            st_z(3)*fov_length_att, 'Color', color_st, ...  % color_st 사용
            'LineWidth', 4, 'MaxHeadSize', 0.5);
    text(st_z(1)*fov_length_att*1.05, st_z(2)*fov_length_att*1.05, ...
         st_z(3)*fov_length_att*1.05, 'Star Tracker', ...
         'Color', color_st, 'FontWeight', 'bold', 'FontSize', 10);  % color_st 사용
    
    % Star Tracker FOV Cone
    cone_radius_st_att = fov_length_att * tan(fov_st / 2);
    circle_st_local_att = [cone_radius_st_att * cos(cone_theta); 
                           cone_radius_st_att * sin(cone_theta); 
                           fov_length_att * ones(1, length(cone_theta))];
    circle_st_lvlh_att = R_ST2L * circle_st_local_att;
    
    for i = 1:4:length(cone_theta)
        plot3([0, circle_st_lvlh_att(1,i)], [0, circle_st_lvlh_att(2,i)], ...
              [0, circle_st_lvlh_att(3,i)], 'Color', [color_st 0.3], 'LineWidth', 0.5);  % color_st 사용
    end
    plot3(circle_st_lvlh_att(1,:), circle_st_lvlh_att(2,:), circle_st_lvlh_att(3,:), ...
          'Color', color_st, 'LineWidth', 2);  % color_st 사용
    
    hold off;
    axis equal; grid on;
    xlim([-1.6, 1.6]); ylim([-1.6, 1.6]); zlim([-1.6, 1.6]);
    xlabel('LVLH X (Radial)', 'FontSize', 11, 'FontWeight', 'bold'); 
    ylabel('LVLH Y (In-track)', 'FontSize', 11, 'FontWeight', 'bold'); 
    zlabel('LVLH Z (Cross-track)', 'FontSize', 11, 'FontWeight', 'bold');
    view(45, 25);
    
    title(sprintf(['Deputy Attitude: Camera & Star Tracker\n' ...
                   't = %.1f s | Roll: %.2f° | Pitch: %.2f° | Yaw: %.2f°\n' ...
                   'Camera FOV: %.1f° | ST FOV: %.1f°'], ...
            current_time, euler(1), euler(2), euler(3), ...
            rad2deg(fov_camera), rad2deg(fov_st)), ...
            'FontSize', 13, 'FontWeight', 'bold');
    
    % 전체 제목
    sgtitle(sprintf('Integrated Sensor Model (t = %.1f s, rotation: %.1f deg/s)', ...
                    current_time, rotation_rate_dps), ...
            'FontSize', 16, 'FontWeight', 'bold');
    
    drawnow;
    
    % 센서 상태 출력
    fprintf('t=%.1f s (Camera measurement): ', current_time);
    
    % 카메라 픽셀 오차
    both_visible = (visible_true > 0.5) & (visible_measured > 0.5);
    if sum(both_visible) > 0
        pixel_error = pixels_measured(:, both_visible) - pixels_true(:, both_visible);
        rms_error = sqrt(mean(pixel_error(:).^2));
        fprintf('Camera RMS: %.3f px\n', rms_error);
    end
end

fprintf('애니메이션 완료!\n');
end