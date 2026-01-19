function earth_image = render_earth_camera_view2(r_deputy_eci, q_I2B, K, camera_params, R_e, earth_texture)
% 오프스크린 3D 렌더링: Deputy 카메라 관점에서 본 지구

img_width = camera_params.intrinsic.image_width;
img_height = camera_params.intrinsic.image_height;

% Figure 생성 - Units를 'pixels'로 명시하고 화면 내 위치로 조정
fig_offscreen = figure('Visible', 'off', ...
    'Renderer', 'opengl', ...
    'Position', [100, 100, img_width, img_height], ...  % 화면 내 위치로 변경
    'Units', 'pixels', ...  % 픽셀 단위 명시
    'Color', 'k', ...
    'PaperPositionMode', 'auto', ...
    'InvertHardcopy', 'off');

ax = axes('Parent', fig_offscreen, ...
    'Position', [0, 0, 1, 1], ...
    'Units', 'normalized', ...  % normalized 명시
    'Color', 'k', ...
    'Clipping', 'on');

hold(ax, 'on');

% 지구 렌더링
[x_sph, y_sph, z_sph] = sphere(100);
earth_surf = surf(ax, x_sph*R_e, y_sph*R_e, z_sph*R_e, ...
    'FaceColor', 'texturemap', ...
    'CData', earth_texture, ...
    'EdgeColor', 'none', ...
    'FaceLighting', 'gouraud', ...
    'AmbientStrength', 0.3, ...
    'DiffuseStrength', 0.8, ...
    'SpecularStrength', 0.2);

light('Parent', ax, 'Position', [1, 0.5, 1], 'Style', 'infinite', 'Color', [1 1 0.9]);
light('Parent', ax, 'Position', [-1, -0.5, -0.5], 'Style', 'infinite', 'Color', [0.2 0.2 0.3]);
material(ax, 'dull');

% 카메라 자세 설정
R_I2B = GetDCM_QUAT(q_I2B);
R_B2I = R_I2B';

cam_x_B = [1; 0; 0];
cam_y_B = [0; 0; 1];
cam_z_B = [0; 1; 0];

cam_x_I = R_B2I * cam_x_B;
cam_y_I = R_B2I * cam_y_B;
cam_z_I = R_B2I * cam_z_B;

campos(ax, r_deputy_eci');
camtarget(ax, (r_deputy_eci - cam_z_I * 1.0)');
camup(ax, -cam_y_I');

% FOV 설정
f_y = K(2,2);
fov_vertical_rad = 2 * atan(img_height / (2 * f_y));
fov_vertical_deg = fov_vertical_rad * 180 / pi;
camva(ax, fov_vertical_deg);
camproj(ax, 'perspective');

axis(ax, 'equal');
axis(ax, 'off');

set(ax, 'XLim', [-R_e*2, R_e*2], ...
    'YLim', [-R_e*2, R_e*2], ...
    'ZLim', [-R_e*2, R_e*2]);

% 렌더링 완료 대기 (중요!)
drawnow;
pause(0.05);  % 안정화 시간 추가

% getframe 호출
try
    frame = getframe(ax);
    earth_image = frame.cdata;
catch ME
    % getframe 실패 시 대체 방법
    warning('getframe 실패, print 방식으로 대체: %s', ME.message);
    
    % 대체 방법: print 함수 사용
    temp_file = tempname;
    print(fig_offscreen, temp_file, '-dpng', sprintf('-r%d', 96));
    earth_image = imread([temp_file '.png']);
    delete([temp_file '.png']);
end

% 이미지 크기 조정
if size(earth_image, 1) ~= img_height || size(earth_image, 2) ~= img_width
    earth_image = imresize(earth_image, [img_height, img_width]);
end

close(fig_offscreen);
end