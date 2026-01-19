function ani_eci_simple2(r_A_I, r_B_I, colors, R_e, t1, ani_dt)

if nargin < 6
    ani_dt = 2.0;
end

%% ========== 하드코딩된 파라미터 ==========
epoch_jd = 2460508.5;  % 2027-07-15 00:00 UTC (MLTDN 10:30 기준)

%% ========== 경고 억제 ==========
warning('off', 'MATLAB:HandleGraphics:ObsoletedProperty:JavaFrame');
warning('off', 'MATLAB:ui:javacomponent:FunctionToBeRemoved');
warning('off', 'MATLAB:opengl:deprecated');
warning('off', 'MATLAB:opengl:OpenGLWillBeRemoved');

%% ========== OpenGL 정보 ==========
fprintf('=== OpenGL 하드웨어 가속 확인 ===\n');
gl_data = opengl('data');
fprintf('Version: %s\n', gl_data.Version);
fprintf('Vendor: %s\n', gl_data.Vendor);
fprintf('Renderer: %s\n', gl_data.Renderer);

if gl_data.Software
    fprintf('상태: ✗ SOFTWARE (성능 저하)\n');
else
    fprintf('상태: ✓ HARDWARE 가속\n');
end
fprintf('\n');

%% ========== 텍스처 로드 ==========
obj_folder = 'C:\Users\USER\Desktop\relative2\m3d_src\earth_obj\';
tex_path = [obj_folder 'texture1.jpg'];
earth_texture = imread(tex_path);
earth_texture = imresize(earth_texture, 0.5);
earth_texture = flipud(earth_texture);

omega_earth_dps = 360 / 86400;  % deg/s

%% ========== 프레임 계산 ==========
sim_dt = t1(2) - t1(1);
skip_frame = max(1, round(ani_dt / sim_dt));
fprintf('애니메이션 간격: %.2f s (skip=%d frames)\n\n', ani_dt, skip_frame);

%% ========== Figure 생성 ==========
fig = figure('Name', 'ECI Animation - Dual View', ...
             'Position', [100, 100, 1600, 700], ...
             'Color', 'k', ...
             'Renderer', 'opengl', ...
             'DoubleBuffer', 'on', ...
             'GraphicsSmoothing', 'on');

fprintf('ECI 애니메이션 시작...\n');

%% ========== 서브플롯 1 생성 ==========
ax1 = subplot(1, 2, 1, 'Parent', fig);
set(ax1, 'Color', 'k', 'SortMethod', 'depth', 'Clipping', 'on');
hold(ax1, 'on');

[h1, earth_transform1] = create_scene(ax1, r_A_I, r_B_I, colors, R_e, ...
                                       earth_texture, t1, epoch_jd);
view(ax1, -127, 21);
title(ax1, 'View 1: Oblique', 'FontSize', 12, 'FontWeight', 'bold', 'Color', 'w');

%% ========== 서브플롯 2 생성 ==========
ax2 = subplot(1, 2, 2, 'Parent', fig);
set(ax2, 'Color', 'k', 'SortMethod', 'depth', 'Clipping', 'on');
hold(ax2, 'on');

[h2, earth_transform2] = create_scene(ax2, r_A_I, r_B_I, colors, R_e, ...
                                       earth_texture, t1, epoch_jd);
view(ax2, -90, 0);
title(ax2, 'View 2: Top', 'FontSize', 12, 'FontWeight', 'bold', 'Color', 'w');

%% ========== 전역 타이틀 ==========
h_title = sgtitle(sprintf('t = %.1f s', t1(1)), ...
                  'FontSize', 14, 'FontWeight', 'bold', 'Color', 'w');

%% ========== 성능 모니터링 ==========
frame_times = zeros(1, ceil(length(t1)/skip_frame));
frame_idx = 0;

%% ========== 애니메이션 루프 ==========
tic;
for k = 1:skip_frame:length(t1)
    frame_idx = frame_idx + 1;
    frame_tic = tic;
    
    % 태양 위치 계산
    sun_eci_k = compute_sun_position_eci(t1(k), epoch_jd);
    sun_dir_k = sun_eci_k / norm(sun_eci_k);
    
    % 지구 회전각
    rotation_angle_rad = deg2rad(omega_earth_dps * t1(k));
    Rz = makehgtform('zrotate', rotation_angle_rad);
    
    k_end = min(k, length(t1));
    
    % 서브플롯 1 업데이트
    update_scene(h1, earth_transform1, r_A_I, r_B_I, k_end, sun_dir_k, Rz, colors);
    
    % 서브플롯 2 업데이트
    update_scene(h2, earth_transform2, r_A_I, r_B_I, k_end, sun_dir_k, Rz, colors);
    
    set(h_title, 'String', sprintf('t = %.1f s', t1(k_end)));
    
    drawnow limitrate;
    
    frame_times(frame_idx) = toc(frame_tic);
    
    if mod(frame_idx, 10) == 1
        avg_fps = 1 / mean(frame_times(max(1,frame_idx-10):frame_idx));
        fprintf('t=%.1f s | k=%d/%d | FPS: %.1f\n', ...
                t1(k_end), k, length(t1), avg_fps);
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

%% ========== 서브함수: 씬 생성 ==========
function [handles, earth_transform] = create_scene(ax, r_A_I, r_B_I, colors, ...
                                                    R_e, earth_texture, t1, epoch_jd)
    
    [x, y, z] = sphere(100);
    
    % 초기 태양 위치
    sun_eci_0 = compute_sun_position_eci(t1(1), epoch_jd);
    sun_dir_0 = sun_eci_0 / norm(sun_eci_0);
    
    % 태양 광원
    h_sun_light = light('Parent', ax, 'Position', sun_dir_0, ...
                         'Style', 'infinite', 'Color', [1 1 0.95]);
    light('Parent', ax, 'Position', -sun_dir_0*0.2, ...
          'Style', 'infinite', 'Color', [0.15 0.15 0.2]);
    
    % 지구 hgtransform
    earth_transform = hgtransform('Parent', ax);
    
    surf(x*R_e, y*R_e, z*R_e, ...
        'Parent', earth_transform, ...
        'FaceColor', 'texturemap', ...
        'CData', earth_texture, ...
        'EdgeColor', 'none', ...
        'FaceLighting', 'gouraud', ...
        'AmbientStrength', 0.3, ...
        'DiffuseStrength', 0.8, ...
        'SpecularStrength', 0.2);
    
    material(ax, 'dull');
    
    % 궤도 전체
    plot3(ax, r_A_I(1,:), r_A_I(2,:), r_A_I(3,:), ...
        'Color', colors.chief, 'LineWidth', 2);
    plot3(ax, r_B_I(1,:), r_B_I(2,:), r_B_I(3,:), ...
        'Color', colors.deputy, 'LineWidth', 1.5, 'LineStyle', '--');
    
    % 현재 궤적
    h_chief_current = plot3(ax, r_A_I(1,1), r_A_I(2,1), r_A_I(3,1), ...
        'Color', colors.chief, 'LineWidth', 3);
    h_deputy_current = plot3(ax, r_B_I(1,1), r_B_I(2,1), r_B_I(3,1), ...
        'Color', colors.deputy, 'LineWidth', 3);
    
    % 현재 위치
    h_chief_pos = plot3(ax, r_A_I(1,1), r_A_I(2,1), r_A_I(3,1), ...
        'o', 'Color', colors.chief, 'MarkerFaceColor', colors.chief, 'MarkerSize', 10);
    h_deputy_pos = plot3(ax, r_B_I(1,1), r_B_I(2,1), r_B_I(3,1), ...
        's', 'Color', colors.deputy, 'MarkerFaceColor', colors.deputy, 'MarkerSize', 10);
    
    % Axes 설정
    grid(ax, 'on');
    axis(ax, 'equal');
    xlabel(ax, 'ECI X [km]', 'FontSize', 9);
    ylabel(ax, 'ECI Y [km]', 'FontSize', 9);
    zlabel(ax, 'ECI Z [km]', 'FontSize', 9);
    set(ax, 'FontSize', 9, 'XColor', 'w', 'YColor', 'w', 'ZColor', 'w');
    
    % 핸들 저장
    handles.sun_light = h_sun_light;
    handles.chief_current = h_chief_current;
    handles.deputy_current = h_deputy_current;
    handles.chief_pos = h_chief_pos;
    handles.deputy_pos = h_deputy_pos;
end

%% ========== 서브함수: 씬 업데이트 ==========
function update_scene(handles, earth_transform, r_A_I, r_B_I, k, sun_dir, Rz, colors)
    
    % 태양 광원 업데이트
    set(handles.sun_light, 'Position', sun_dir);
    
    % 지구 회전
    set(earth_transform, 'Matrix', Rz);
    
    % 궤적 업데이트
    set(handles.chief_current, 'XData', r_A_I(1,1:k), ...
                                'YData', r_A_I(2,1:k), ...
                                'ZData', r_A_I(3,1:k));
    set(handles.deputy_current, 'XData', r_B_I(1,1:k), ...
                                 'YData', r_B_I(2,1:k), ...
                                 'ZData', r_B_I(3,1:k));
    
    % 현재 위치
    set(handles.chief_pos, 'XData', r_A_I(1,k), ...
                           'YData', r_A_I(2,k), ...
                           'ZData', r_A_I(3,k));
    set(handles.deputy_pos, 'XData', r_B_I(1,k), ...
                            'YData', r_B_I(2,k), ...
                            'ZData', r_B_I(3,k));
end

%% ========== 서브함수: 태양 위치 계산 ==========
function sun_eci = compute_sun_position_eci(t_sec, epoch_jd)
    days_since_epoch = t_sec / 86400;
    jd = epoch_jd + days_since_epoch;
    
    n = jd - 2451545.0;
    L = mod(280.460 + 0.9856474 * n, 360);
    g = mod(357.528 + 0.9856003 * n, 360);
    
    lambda = L + 1.915*sind(g) + 0.020*sind(2*g);
    epsilon = 23.439 - 0.0000004*n;
    
    AU_km = 149597870.7;
    sun_eci = AU_km * [cosd(lambda); 
                       sind(lambda)*cosd(epsilon); 
                       sind(lambda)*sind(epsilon)];
end