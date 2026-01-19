%% Star Tracker 애니메이션 (ani_3d_lvlh_formation_v2 스타일)
function ani_star_tracker(star_data_all, sim_data, colors, skip_frame)
% Star Tracker 애니메이션
% 좌측: 3D LVLH 뷰 + Deputy + Star Tracker FOV
% 우측: Star Tracker 이미지 (별)

if nargin < 4
    skip_frame = 50;  % 기본값
end

% %% 센서 파라미터
% l = 3280;
% w = 2464;
% myu = 1.12e-6;
% f = 0.00304;

%% 센서 파라미터 - 1280x720 모드
myu = 2e-6;             % 픽셀 크기 (m) - OV4689: 2μm x 2μm
f = 0.01042;            % 초점거리 (m) - FOV 14도 기준 10.42mm
l = 1280;               % 가로 픽셀 수
w = 720;                % 세로 픽셀 수

% %% 센서 파라미터 - 2688x1520 최대 해상도
% myu = 2e-6;             % 픽셀 크기 (m) - OV4689: 2μm x 2μm
% f = 0.02188;            % 초점거리 (m) - FOV 14도 기준 21.88mm
% l = 2688;               % 가로 픽셀 수
% w = 1520;               % 세로 픽셀 수

FOVx = rad2deg(2 * atan((myu*l/2) / f));
FOVy = rad2deg(2 * atan((myu*w/2) / f));
fov_rad = sqrt(deg2rad(FOVx)^2 + deg2rad(FOVy)^2) / 2;

fprintf('Star Tracker 애니메이션 시작...\n');
fprintf('  FOV: %.1f° x %.1f°\n', FOVx, FOVy);
fprintf('  Skip: %d 프레임\n\n', skip_frame);

%% Figure 생성
fig = figure('Name', 'Star Tracker Animation', ...
             'Position', [50, 50, 1900, 950], ...
             'Color', 'k');

%% 데이터 추출
LVLH_pos = sim_data.traj.r_B_L;
time = sim_data.time.t_s;
quat_I2B = sim_data.quat.q_I2B;
quat_L2B = sim_data.quat.q_L2B;
r_init_B = norm(LVLH_pos(:, 1));

% Star Tracker 방향
R_B2ST = [1, 0, 0; 0, 0, 1; 0,-1, 0];

% 색상
if nargin < 3
    colors = struct('deputy', [0.6350, 0.0780, 0.1840], ...
                    'sat_body_x', [1, 0, 0], ...
                    'sat_body_y', [0, 1, 0], ...
                    'sat_body_z', [0, 0, 1], ...
                    'st_axis', [0.2, 0.6, 1.0]);
end

%% 별 렌더링 함수 (레퍼런스 기반)
    function background = render_stars(pixel_coords, magnitudes, width, height)
        background = zeros(height, width, 'uint8');
        
        for i = 1:size(pixel_coords, 1)
            x = round(width/2 + pixel_coords(i, 1));
            y = round(height/2 - pixel_coords(i, 2));
            
            if x < 1 || x > width || y < 1 || y > height
                continue;
            end
            
            mag = abs(magnitudes(i) - 7);
            radius = round((mag/9)*5 + 2);
            color = round((mag/9)*155 + 100);
            
            % 원 그리기
            for dx = -radius:radius
                for dy = -radius:radius
                    if dx^2 + dy^2 <= radius^2
                        px = x + dx;
                        py = y + dy;
                        
                        if px >= 1 && px <= width && py >= 1 && py <= height
                            background(py, px) = max(background(py, px), color);
                        end
                    end
                end
            end
        end
        
        % 노이즈 추가
        noise = randi([0, 50], height, width);
        background = uint8(0.9 * double(background) + 0.1 * double(noise));
    end

%% 애니메이션 루프
for k = 1:skip_frame:length(star_data_all)
    
    frame_idx = star_data_all(k).frame;
    current_time = star_data_all(k).time;
    current_pos = LVLH_pos(:, frame_idx);
    
    % 자세
    q_L2B = quat_L2B(:, frame_idx);
    DCM_L2B = GetDCM_QUAT(q_L2B);
    R_B2L = DCM_L2B';
    
    % Body 축 (LVLH)
    body_x_L = R_B2L * [1; 0; 0];
    body_y_L = R_B2L * [0; 1; 0];
    body_z_L = R_B2L * [0; 0; 1];
    
    % Star Tracker 축 (LVLH)
    st_x_L = R_B2L * R_B2ST' * [1; 0; 0];
    st_y_L = R_B2L * R_B2ST' * [0; 1; 0];
    st_z_L = R_B2L * R_B2ST' * [0; 0; 1];  % 광축
    
    %% 좌측: 3D LVLH 뷰
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
    
    % Chief 위치
    plot3(0, 0, 0, 'yo', 'MarkerSize', 12, 'MarkerFaceColor', 'y');
    
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
    
    % Star Tracker 광축
    fov_length = r_init_B * 0.8;
    quiver3(current_pos(1), current_pos(2), current_pos(3), ...
            st_z_L(1)*fov_length, st_z_L(2)*fov_length, st_z_L(3)*fov_length, ...
            'Color', colors.st_axis, 'LineWidth', 4.5, 'MaxHeadSize', 0.4);
    
    % FOV Cone
    cone_theta = linspace(0, 2*pi, 24);
    cone_radius = fov_length * tan(fov_rad);
    circle_st = [cone_radius * cos(cone_theta); 
                 cone_radius * sin(cone_theta); 
                 fov_length * ones(1, length(cone_theta))];
    R_ST2L = [st_x_L, st_y_L, st_z_L];
    circle_st_lvlh = R_ST2L * circle_st + current_pos;
    plot3(circle_st_lvlh(1,:), circle_st_lvlh(2,:), circle_st_lvlh(3,:), ...
          'Color', colors.st_axis, 'LineWidth', 2);
    
    % 궤도
    plot3(LVLH_pos(1, 1:frame_idx), LVLH_pos(2, 1:frame_idx), LVLH_pos(3, 1:frame_idx), ...
          'Color', colors.deputy, 'LineWidth', 2);
    plot3(current_pos(1), current_pos(2), current_pos(3), 'mo', ...
          'MarkerSize', 10, 'MarkerFaceColor', 'm');
    
    hold off;
    axis equal; grid on;
    lim = r_init_B;
    xlim([-lim, lim]); ylim([-lim, lim]); zlim([-lim, lim]);
    xlabel('Radial (x) [km]', 'Color', 'w');
    ylabel('In-track (y) [km]', 'Color', 'w');
    zlabel('Cross-track (z) [km]', 'Color', 'w');
    set(gca, 'XColor', 'w', 'YColor', 'w', 'ZColor', 'w');
    view(-45, 25);
    title(sprintf('3D LVLH: %d Stars Visible', star_data_all(k).num_stars), 'Color', 'w');
    
    %% 우측: Star Tracker 이미지
    subplot(1, 2, 2);
    cla;
    
    % 별 렌더링
    pixel_coords = star_data_all(k).pixel_coords;
    magnitudes = star_data_all(k).magnitudes;
    star_image = render_stars(pixel_coords, magnitudes, l, w);
    
    imshow(star_image);
    hold on;
    
    % 프레임
    rectangle('Position', [0, 0, l, w], 'EdgeColor', 'w', 'LineWidth', 2.5);
    
    % 중심 십자
    % plot(l/2, w/2, 'w+', 'MarkerSize', 20, 'LineWidth', 0.1);
    
    hold off;
    xlim([0, l]); ylim([0, w]);
    set(gca, 'YDir', 'reverse');
    axis equal; grid on;
    xlabel('u [pixels]', 'Color', 'w');
    ylabel('v [pixels]', 'Color', 'w');
    title(sprintf('Star Tracker Image\nRA: %.1f°, DEC: %.1f°', ...
          rad2deg(star_data_all(k).ra), rad2deg(star_data_all(k).dec)), 'Color', 'w');
    
    sgtitle(sprintf('t = %.1f s (Frame %d/%d)', ...
            current_time, k, length(star_data_all)), ...
            'FontSize', 15, 'FontWeight', 'bold', 'Color', 'w');
    
    drawnow;
    
    if mod(k, skip_frame*5) == 1
        fprintf('t=%.1f s: %d stars\n', current_time, star_data_all(k).num_stars);
    end
end

fprintf('애니메이션 완료!\n');
end