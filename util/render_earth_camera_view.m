function earth_image = render_earth_camera_view(r_deputy_eci, q_I2B, K, camera_params, R_e, earth_texture)

img_width = camera_params.intrinsic.image_width;
img_height = camera_params.intrinsic.image_height;

fig_offscreen = figure('Visible', 'off', ...
    'Renderer', 'opengl', ...
    'Position', [0, 0, img_width, img_height], ...
    'Color', 'k');
ax = axes('Parent', fig_offscreen, ...
    'Position', [0, 0, 1, 1], ...
    'Color', 'k', ...
    'Clipping', 'on');
hold(ax, 'on');

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

R_I2B = GetDCM_QUAT(q_I2B);
R_B2I = R_I2B';

cam_x_B = [1; 0; 0];
cam_y_B = [0; 0; -1];
cam_z_B = [0; 1; 0];  % Body Y축 = Chief pointing

cam_x_I = R_B2I * cam_x_B;
cam_y_I = R_B2I * cam_y_B;
cam_z_I = R_B2I * cam_z_B;

campos(ax, r_deputy_eci');

% === 핵심 수정: 부호 반전 ===
camtarget(ax, (r_deputy_eci + cam_z_I * 100)');  % - 를 + 로!

camup(ax, cam_y_I');  % 부호 제거

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

drawnow;
frame = getframe(ax);
earth_image = frame.cdata;

if size(earth_image, 1) ~= img_height || size(earth_image, 2) ~= img_width
    earth_image = imresize(earth_image, [img_height, img_height]);
end

close(fig_offscreen);
end