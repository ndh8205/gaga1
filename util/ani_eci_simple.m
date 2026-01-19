function ani_eci_simple(r_A_I, r_B_I, colors, R_e, t1, ani_dt)

if nargin < 6
    ani_dt = 2.0;
end

%% ========== 경고 억제 ==========
% opengl deprecated 경고 끄기
warning('off', 'MATLAB:HandleGraphics:ObsoletedProperty:JavaFrame');
warning('off', 'MATLAB:ui:javacomponent:FunctionToBeRemoved');

% opengl 경고 ID 확인 및 억제
w = warning('query', 'last');
[~, warnId] = lastwarn;
if ~isempty(warnId)
    warning('off', warnId);
end

% 일반적인 opengl 경고 ID들 모두 억제
warning('off', 'MATLAB:opengl:deprecated');
warning('off', 'MATLAB:opengl:OpenGLWillBeRemoved');

%% ========== OpenGL 정보 (경고 없이) ==========
fprintf('=== OpenGL 하드웨어 가속 확인 ===\n');

gl_data = opengl('data');
fprintf('Version: %s\n', gl_data.Version);
fprintf('Vendor: %s\n', gl_data.Vendor);
fprintf('Renderer: %s\n', gl_data.Renderer);
fprintf('Max Texture Size: %d\n', gl_data.MaxTextureSize);

if gl_data.Software
    fprintf('상태: ✗ SOFTWARE (성능 저하)\n');
else
    fprintf('상태: ✓ HARDWARE 가속\n');
end

if isfield(gl_data, 'HardwareSupportLevel')
    fprintf('Support: %s\n', gl_data.HardwareSupportLevel);
end

fprintf('\n');

%% ========== 텍스처 로드 및 캐싱 ==========
obj_folder = 'C:\Users\USER\Desktop\relative2\m3d_src\earth_obj\';
tex_path = [obj_folder 'texture1.jpg'];
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
fig = figure('Name', 'ECI Animation - Hardware Accelerated', ...
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

fprintf('ECI 애니메이션 시작...\n');

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

%% ========== 궤도 그리기 ==========
h_chief_full = plot3(r_A_I(1,:), r_A_I(2,:), r_A_I(3,:), ...
    'Color', colors.chief, 'LineWidth', 2, 'DisplayName', 'Chief Orbit');
h_deputy_full = plot3(r_B_I(1,:), r_B_I(2,:), r_B_I(3,:), ...
    'Color', colors.deputy, 'LineWidth', 1.5, 'LineStyle', '--', ...
    'DisplayName', 'Deputy Orbit');

h_chief_current = plot3(r_A_I(1,1), r_A_I(2,1), r_A_I(3,1), ...
    'Color', colors.chief, 'LineWidth', 3);
h_deputy_current = plot3(r_B_I(1,1), r_B_I(2,1), r_B_I(3,1), ...
    'Color', colors.deputy, 'LineWidth', 3);

h_chief_pos = plot3(r_A_I(1,1), r_A_I(2,1), r_A_I(3,1), ...
    'o', 'Color', colors.chief, 'MarkerFaceColor', colors.chief, ...
    'MarkerSize', 10, 'DisplayName', 'Chief');
h_deputy_pos = plot3(r_B_I(1,1), r_B_I(2,1), r_B_I(3,1), ...
    's', 'Color', colors.deputy, 'MarkerFaceColor', colors.deputy, ...
    'MarkerSize', 10, 'DisplayName', 'Deputy');

%% ========== Axes 설정 ==========
grid on;
axis equal;
title('ECI Frame - Chief and Deputy Orbits', 'FontSize', 12, 'FontWeight', 'bold');
xlabel('ECI X [km]', 'FontSize', 10);
ylabel('ECI Y [km]', 'FontSize', 10);
zlabel('ECI Z [km]', 'FontSize', 10);
legend('Location', 'best', 'FontSize', 9, 'TextColor', 'w', ...
       'Color', [0.1 0.1 0.1], 'EdgeColor', 'w');
view(45, 20);
set(gca, 'FontSize', 9, 'XColor', 'w', 'YColor', 'w', 'ZColor', 'w');

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
    
    rotation_angle = omega_earth * t1(k);
    cache_idx = max(1, min(num_texture_cache, ...
                    round((rotation_angle / max_rotation) * num_texture_cache)));
    set(earth_surf, 'CData', texture_cache{cache_idx});
    
    k_end = min(k, length(t1));
    set(h_chief_current, 'XData', r_A_I(1,1:k_end), ...
                         'YData', r_A_I(2,1:k_end), ...
                         'ZData', r_A_I(3,1:k_end));
    set(h_deputy_current, 'XData', r_B_I(1,1:k_end), ...
                          'YData', r_B_I(2,1:k_end), ...
                          'ZData', r_B_I(3,1:k_end));
    
    set(h_chief_pos, 'XData', r_A_I(1,k_end), ...
                     'YData', r_A_I(2,k_end), ...
                     'ZData', r_A_I(3,k_end));
    set(h_deputy_pos, 'XData', r_B_I(1,k_end), ...
                      'YData', r_B_I(2,k_end), ...
                      'ZData', r_B_I(3,k_end));
    
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

%% ========== 경고 복원 (선택) ==========
% warning('on', 'all');  % 필요시 활성화

end