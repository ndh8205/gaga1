%% 리얼한 지구 시각화 (검증된 기능만 사용)
clc; clear; close all;

obj_folder = 'C:\Users\USER\Desktop\relative2\m3d_src\earth_obj\';
tex_path = [obj_folder 'texture1.jpg'];
R_e = 6378.136;

fprintf('texture1.jpg 로드...\n');
earth_texture = imread(tex_path);
fprintf('텍스처 크기: %d x %d\n', size(earth_texture, 1), size(earth_texture, 2));

%% 부드러운 구체 생성
[x, y, z] = sphere(150);

%% Figure 설정 - 검은 배경
figure('Position', [100, 100, 800, 800], 'Color', 'k');
ax = axes('Color', 'k');
hold on;

%% 지구 렌더링
earth = surf(x*R_e, y*R_e, z*R_e, ...
    'FaceColor', 'texturemap', ...
    'CData', flipud(earth_texture), ...
    'EdgeColor', 'none', ...
    'FaceLighting', 'gouraud', ...
    'AmbientStrength', 0.3, ...
    'DiffuseStrength', 0.8, ...
    'SpecularStrength', 0.2, ...
    'SpecularExponent', 5);

%% 축과 그리드 제거
axis equal off;

%% 조명 설정 (태양광 효과)
light('Position', [1, 0.5, 1], 'Style', 'infinite', 'Color', [1 1 0.9]);
light('Position', [-1, -0.5, -0.5], 'Style', 'infinite', 'Color', [0.2 0.2 0.3]);

%% 카메라 설정
view(45, 20);
camzoom(1.2);

fprintf('완료!\n');