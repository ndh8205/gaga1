function ani_eci_deputy_fixed_lvlh(r_A_I, r_B_I, v_A_I, sensor_data, colors, R_e, t1, ani_dt)
% Chief LVLH 좌표계 기준 뷰 (OpenGL 최적화)

if nargin < 8
    ani_dt = 2.0;
end

%% ========== 경고 억제 ==========
warning('off', 'MATLAB:HandleGraphics:ObsoletedProperty:JavaFrame');
warning('off', 'MATLAB:ui:javacomponent:FunctionToBeRemoved');
[~, warnId] = lastwarn;
if ~isempty(warnId)
    warning('off', warnId);
end
warning('off', 'MATLAB:opengl:deprecated');
warning('off', 'MATLAB:opengl:OpenGLWillBeRemoved');

%% ========== OpenGL 정보 ==========
fprintf('=== OpenGL 하드웨어 가속 확인 ===\n');
gl_data = opengl('data');
fprintf('Version: %s\n', gl_data.Version);
fprintf('Vendor: %s\n', gl_data.Vendor);
fprintf('Renderer: %s\n', gl_data.Renderer);

if gl_data.Software
    fprintf('상태: ✗ SOFTWARE\n');
else
    fprintf('상태: ✓ HARDWARE 가속\n');
end
fprintf('\n');

%% ========== 텍스처 로드 및 캐싱 ==========
obj_folder = 'C:\Users\USER\Desktop\relative2\m3d_src\earth_obj\';
tex_path = [obj_folder 'texture2.jpg'];
earth_texture = imread(tex_path);
earth_texture = imresize(earth_texture, 0.5);
earth_texture = flipud(earth_texture);

omega_earth = 360 / 86400;
max_rotation = omega_earth * t1(end);
num_texture_cache = min(360, ceil(max_rotation));
fprintf('텍스처 캐시 생성 중 (%d 프레임)...\n', num_texture_cache);

texture_cache = cell(1, num_texture_cache);
for i = 1:num_texture_cache
    angle = (i-1) / num_texture_cache * max_rotation;
    shift_px = round((angle / 360) * size(earth_texture, 2));
    texture_cache{i} = circshift(earth_texture, [0, shift_px]);
end
fprintf('캐시 완료.\n');

%% ========== 프레임 계산 ==========
sim_dt = t1(2) - t1(1);
skip_frame = max(1, round(ani_dt / sim_dt));
fprintf('애니메이션 간격: %.2f s (skip=%d frames)\n\n', ani_dt, skip_frame);

%% ========== Figure 생성 ==========
fig = figure('Name', 'ECI Animation - Chief LVLH View', ...
             'Position', [100, 100, 1200, 800], ...
             'Color', 'k', ...
             'Renderer', 'opengl', ...
             'DoubleBuffer', 'on', ...
             'GraphicsSmoothing', 'on');

ax = axes('Parent', fig, ...
          'Color', 'k', ...
          'SortMethod', 'depth', ...
          'Clipping', 'on');
hold on;

fprintf('ECI 애니메이션 시작 (Chief LVLH 추적)...\n');

%% ========== 지구 생성 ==========
[x, y, z] = sphere(100);

light('Position', [1, 0.5, 1], 'Style', 'infinite', 'Color', [1 1 0.9]);
light('Position', [-1, -0.5, -0.5], 'Style', 'infinite', 'Color', [0.2 0.2 0.3]);

earth_surf = surf(x*R_e, y*R_e, z*R_e, ...
    'FaceColor', 'texturemap', ...
    'CData', earth_texture, ...
    'EdgeColor', 'none', ...
    'FaceLighting', 'gouraud', ...
    'AmbientStrength', 0.3, ...
    'DiffuseStrength', 0.8, ...
    'SpecularStrength', 0.2);

material(ax, 'dull');

% 지구 경계선 강조
limb_sphere = surf(x*R_e*1.001, y*R_e*1.001, z*R_e*1.001, ...
    'FaceColor', 'none', ...
    'EdgeColor', [0.4, 0.6, 0.9], ...
    'LineWidth', 0.8, ...
    'EdgeAlpha', 0.4);

%% ========== 궤도 그리기 ==========
h_chief_full = plot3(r_A_I(1,:), r_A_I(2,:), r_A_I(3,:), ...
    'Color', [colors.chief 0.3], 'LineWidth', 1);
h_deputy_full = plot3(r_B_I(1,:), r_B_I(2,:), r_B_I(3,:), ...
    'Color', [colors.deputy 0.3], 'LineWidth', 1, 'LineStyle', '--');

h_chief_current = plot3(r_A_I(1,1), r_A_I(2,1), r_A_I(3,1), ...
    'Color', colors.chief, 'LineWidth', 3);
h_deputy_current = plot3(r_B_I(1,1), r_B_I(2,1), r_B_I(3,1), ...
    'Color', colors.deputy, 'LineWidth', 3);

h_chief_pos = plot3(r_A_I(1,1), r_A_I(2,1), r_A_I(3,1), ...
    'o', 'Color', colors.chief, 'MarkerFaceColor', colors.chief, ...
    'MarkerSize', 12, 'LineWidth', 2);
h_deputy_pos = plot3(r_B_I(1,1), r_B_I(2,1), r_B_I(3,1), ...
    's', 'Color', colors.deputy, 'MarkerFaceColor', colors.deputy, ...
    'MarkerSize', 10, 'LineWidth', 2);

%% ========== Body 축 handles ==========
axis_len = 0.02;

% Chief
h_chief_x = quiver3(0,0,0, 1,0,0, 'Color', colors.sat_body_x, 'LineWidth', 2.5, 'MaxHeadSize', 0.5);
h_chief_y = quiver3(0,0,0, 0,1,0, 'Color', colors.sat_body_y, 'LineWidth', 2.5, 'MaxHeadSize', 0.5);
h_chief_z = quiver3(0,0,0, 0,0,1, 'Color', colors.sat_body_z, 'LineWidth', 2.5, 'MaxHeadSize', 0.5);

% Deputy
h_deputy_x = quiver3(0,0,0, 1,0,0, 'Color', colors.sat_body_x, 'LineWidth', 2.5, 'MaxHeadSize', 0.5);
h_deputy_y = quiver3(0,0,0, 0,1,0, 'Color', colors.sat_body_y, 'LineWidth', 2.5, 'MaxHeadSize', 0.5);
h_deputy_z = quiver3(0,0,0, 0,0,1, 'Color', colors.sat_body_z, 'LineWidth', 2.5, 'MaxHeadSize', 0.5);

%% ========== Axes 설정 ==========
grid on;
axis equal;
title('ECI Frame - Chief LVLH Tracking View', 'FontSize', 12, 'FontWeight', 'bold', 'Color', 'w');
xlabel('ECI X [km]', 'FontSize', 10);
ylabel('ECI Y [km]', 'FontSize', 10);
zlabel('ECI Z [km]', 'FontSize', 10);
set(gca, 'FontSize', 9, 'XColor', 'w', 'YColor', 'w', 'ZColor', 'w');
h_title = sgtitle(sprintf('t = %.1f s', t1(1)), 'FontSize', 14, 'FontWeight', 'bold', 'Color', 'w');

%% ========== 성능 모니터링 ==========
frame_times = zeros(1, ceil(length(t1)/skip_frame));
frame_idx = 0;

%% ========== 애니메이션 루프 ==========
tic;
for k = 1:skip_frame:length(t1)
    frame_idx = frame_idx + 1;
    frame_tic = tic;
    
    % 텍스처 캐시
    rotation_angle = omega_earth * t1(k);
    cache_idx = max(1, min(num_texture_cache, ...
                    round((rotation_angle / max_rotation) * num_texture_cache)));
    set(earth_surf, 'CData', texture_cache{cache_idx});
    
    % 궤도 업데이트
    k_end = min(k, length(t1));
    set(h_chief_current, 'XData', r_A_I(1,1:k_end), ...
                         'YData', r_A_I(2,1:k_end), ...
                         'ZData', r_A_I(3,1:k_end));
    set(h_deputy_current, 'XData', r_B_I(1,1:k_end), ...
                          'YData', r_B_I(2,1:k_end), ...
                          'ZData', r_B_I(3,1:k_end));
    
    % 현재 위성 위치
    r_chief = r_A_I(:,k_end);
    r_deputy = r_B_I(:,k_end);
    v_chief = v_A_I(:,k_end);
    
    set(h_chief_pos, 'XData', r_chief(1), 'YData', r_chief(2), 'ZData', r_chief(3));
    set(h_deputy_pos, 'XData', r_deputy(1), 'YData', r_deputy(2), 'ZData', r_deputy(3));
    
    % Chief Body 축 업데이트
    q_I2A = sensor_data(k_end).q_I2A;
    R_I2A = GetDCM_QUAT(q_I2A);
    R_A2I = R_I2A';
    
    set(h_chief_x, 'XData', r_chief(1), 'YData', r_chief(2), 'ZData', r_chief(3), ...
        'UData', R_A2I(1,1)*axis_len, 'VData', R_A2I(2,1)*axis_len, 'WData', R_A2I(3,1)*axis_len);
    set(h_chief_y, 'XData', r_chief(1), 'YData', r_chief(2), 'ZData', r_chief(3), ...
        'UData', R_A2I(1,2)*axis_len, 'VData', R_A2I(2,2)*axis_len, 'WData', R_A2I(3,2)*axis_len);
    set(h_chief_z, 'XData', r_chief(1), 'YData', r_chief(2), 'ZData', r_chief(3), ...
        'UData', R_A2I(1,3)*axis_len, 'VData', R_A2I(2,3)*axis_len, 'WData', R_A2I(3,3)*axis_len);
    
    % Deputy Body 축 업데이트
    q_I2B = sensor_data(k_end).q_I2B;
    R_I2B = GetDCM_QUAT(q_I2B);
    R_B2I = R_I2B';
    
    set(h_deputy_x, 'XData', r_deputy(1), 'YData', r_deputy(2), 'ZData', r_deputy(3), ...
        'UData', R_B2I(1,1)*axis_len, 'VData', R_B2I(2,1)*axis_len, 'WData', R_B2I(3,1)*axis_len);
    set(h_deputy_y, 'XData', r_deputy(1), 'YData', r_deputy(2), 'ZData', r_deputy(3), ...
        'UData', R_B2I(1,2)*axis_len, 'VData', R_B2I(2,2)*axis_len, 'WData', R_B2I(3,2)*axis_len);
    set(h_deputy_z, 'XData', r_deputy(1), 'YData', r_deputy(2), 'ZData', r_deputy(3), ...
        'UData', R_B2I(1,3)*axis_len, 'VData', R_B2I(2,3)*axis_len, 'WData', R_B2I(3,3)*axis_len);
    
    % Chief LVLH 좌표계 계산
    r_hat = r_chief / norm(r_chief);
    h_vec = cross(r_chief, v_chief);
    h_hat = h_vec / norm(h_vec);
    t_hat = cross(h_hat, r_hat);
    
    % 축 범위
    orbit_radius = norm(r_chief);
    scene_range = orbit_radius * 1.1;
    
    xlim([-scene_range, scene_range]);
    ylim([-scene_range, scene_range]);
    zlim([-scene_range, scene_range]);
    
    % LVLH 기준 카메라 위치
    cam_distance = orbit_radius * 0.2;
    cam_pos = r_chief + r_hat * cam_distance * 0.5 ...
                      + t_hat * cam_distance * 0.7 ...
                      + h_hat * cam_distance * 0.4;
    
    campos(cam_pos);
    camtarget(r_chief);
    camup(h_hat);
    camva(0.0195);
    
    % 타이틀
    set(h_title, 'String', sprintf('t = %.1f s | Alt: %.1f km | Earth Rot: %.1f°', ...
        t1(k_end), norm(r_chief) - R_e, mod(rotation_angle, 360)));
    
    drawnow limitrate;
    
    frame_times(frame_idx) = toc(frame_tic);
    
    if mod(frame_idx, 10) == 1
        avg_fps = 1 / mean(frame_times(max(1,frame_idx-10):frame_idx));
        fprintf('t=%.1f s | k=%d/%d | FPS: %.1f | Rot: %.1f°\n', ...
                t1(k_end), k, length(t1), avg_fps, mod(rotation_angle, 360));
    end
end

total_time = toc;

%% ========== 성능 통계 ==========
fprintf('\n=== 애니메이션 완료 ===\n');
fprintf('총 시간: %.2f s\n', total_time);
fprintf('평균 FPS: %.1f\n', frame_idx / total_time);
fprintf('평균: %.1f ms | 중간값: %.1f ms\n', ...
        mean(frame_times) * 1000, median(frame_times) * 1000);
fprintf('최소: %.1f ms | 최대: %.1f ms | 95%%: %.1f ms\n', ...
        min(frame_times) * 1000, max(frame_times) * 1000, ...
        prctile(frame_times, 95) * 1000);

spikes = frame_times > 0.05;
if any(spikes)
    fprintf('⚠ %d개 프레임 스파이크 (>50ms)\n', sum(spikes));
end

end