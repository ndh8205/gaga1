%% 통합 시뮬레이션: KARI 로봇 팔 & 6U / 3U 큐브 위성 (Final Ver)
clear; clc; close all;

% [1] 경로 설정
baseDir = 'D:\pj2025\space_challenge';
addpath(genpath(baseDir)); 

% 파일 경로 정의
robotMeshPath = fullfile(baseDir, 'model', 'modeling_3d', 'ASM_KARI_ARM', 'meshes');
sat6UPath = fullfile(baseDir, 'model', 'modeling_3d', '6U_CubeSat_v7_nopayload', '6U_CubeSat_v7_nopayload.obj');
sat3UPath = fullfile(baseDir, 'model', 'modeling_3d', '1015_3U_CNU', '1015_3U_CNU.obj');

%% [2] 뷰어 및 환경 초기화 (Wide View)
f = figure('Color', 'k', 'Name', 'Space Mission: Dual Satellites', ...
    'InvertHardcopy', 'off', 'Renderer', 'opengl', ...
    'Position', [50 50 1400 900]); % 창 크기를 더 키움

ax = axes('Parent', f, 'Color', 'k', ...
    'XColor', 'w', 'YColor', 'w', 'ZColor', 'w', ...
    'DataAspectRatio', [1 1 1], 'NextPlot', 'add');

grid off; axis off;
view(45, 20);

% [핵심 수정] 화면 잘림 방지를 위해 범위 대폭 확장 (-2.0 ~ 2.0)
xlim([-2.0 2.0]); 
ylim([-2.0 2.0]); 
zlim([-1.0 2.5]); 
axis vis3d manual; 

% 조명 (태양광 + 지구반사광 느낌)
light('Position', [5 2 5], 'Style', 'local', 'Color', [1 1 0.95]); 
light('Position', [-3 -3 0], 'Style', 'local', 'Color', [0.2 0.3 0.5]); 
lighting gouraud;

%% [3] 위성 로드 및 배치
scaleFactor = 0.001; % mm -> m 변환

% --- (A) 6U CubeSat (로봇의 오른쪽 앞) ---
fprintf('Loading 6U CubeSat...\n');
[sat6UData, sat6UMats, sat6UAssign] = readObjWithCache(sat6UPath);
sat6UGroup = hgtransform('Parent', ax);
renderObjToGroup(sat6UGroup, sat6UData, sat6UMats, sat6UAssign);

% 초기 위치: X=0.8, Y=-0.6 (오른쪽), Z=0.6
pos6U = [0.8, -0.6, 0.6]; 
set(sat6UGroup, 'Matrix', makehgtform('translate', pos6U) * makehgtform('scale', scaleFactor));


% --- (B) 3U CubeSat (로봇의 왼쪽 앞) ---
fprintf('Loading 3U CubeSat...\n');
if isfile(sat3UPath)
    [sat3UData, sat3UMats, sat3UAssign] = readObjWithCache(sat3UPath);
    sat3UGroup = hgtransform('Parent', ax);
    renderObjToGroup(sat3UGroup, sat3UData, sat3UMats, sat3UAssign);
    
    % 초기 위치: X=0.8, Y=0.6 (왼쪽), Z=0.4 (약간 낮게)
    pos3U = [0.8, 0.6, 0.4]; 
    set(sat3UGroup, 'Matrix', makehgtform('translate', pos3U) * makehgtform('scale', scaleFactor));
else
    warning('3U 위성 파일이 없습니다: %s', sat3UPath);
end

%% [4] 로봇 팔 로드
fprintf('Loading Robot Arm...\n');
robotDef = {
    'ASM_J0.STL', 0, [0 0 0], [0 0 0]; 
    'ASM_J1.STL', 1, [0 0 0.094], [0 0 1];
    'ASM_J2.STL', 2, [0 0.088 0.105], [0 1 0];
    'ASM_J3.STL', 3, [0 0.104 0.131], [0 0 1];
    'ASM_J4.STL', 4, [0 -0.088 0.8], [0 1 0];
    'ASM_J5.STL', 5, [0 -0.104 0.131], [0 0 1];
    'ASM_J6.STL', 6, [0 -0.071 0.408], [0 1 0];
    'ASM_J7.STL', 7, [0 -0.121 0.088], [0 0 1];
};

links = struct('hg', [], 'axis', [], 'offset', []);
for i = 1:size(robotDef, 1)
    % (이전과 동일한 로봇 로드 로직)
    fileName = robotDef{i,1}; parentIdx = robotDef{i,2};
    offset = robotDef{i,3}; rotAxis = robotDef{i,4};
    fullPath = fullfile(robotMeshPath, fileName);
    
    if parentIdx == 0, links(i).hg = hgtransform('Parent', ax);
    else, links(i).hg = hgtransform('Parent', links(parentIdx).hg); end
    
    set(links(i).hg, 'Matrix', makehgtform('translate', offset));
    tr = stlread(fullPath);
    p = patch('Parent', links(i).hg, 'Faces', tr.ConnectivityList, 'Vertices', tr.Points, ...
        'FaceColor', [0.9 0.9 0.95], 'EdgeColor', 'none', ...
        'FaceLighting', 'gouraud', 'SpecularStrength', 0.6);
    material(p, 'dull');
    links(i).axis = rotAxis; links(i).offset = offset;
end

%% [5] 애니메이션 루프
disp('Simulation Started with 6U & 3U Satellites.');
t = 0;
rate = rateControl(60); 

while ishandle(f)
    t = t + 0.015;
    
    % --- 1. 위성 애니메이션 ---
    % 6U: 천천히 X축 회전
    rot6U = makehgtform('axisrotate', [1 0.2 0], t*0.3);
    float6U = [0, 0, sin(t)*0.03];
    set(sat6UGroup, 'Matrix', makehgtform('translate', pos6U + float6U) * rot6U * makehgtform('scale', scaleFactor));
    
    % 3U: 좀 더 빠르게 Z축 회전 (다른 패턴)
    if exist('sat3UGroup', 'var')
        rot3U = makehgtform('axisrotate', [0 0 1], t*0.8);
        float3U = [0, 0, cos(t*1.2)*0.04]; % 6U와 반박자로 움직임
        set(sat3UGroup, 'Matrix', makehgtform('translate', pos3U + float3U) * rot3U * makehgtform('scale', scaleFactor));
    end
    
    % --- 2. 로봇 팔 제어 (두 위성을 번갈아 보는 동작) ---
    target_switch = sin(t * 0.5); % -1(우측) ~ 1(좌측) 왔다갔다 함
    
    q1 = target_switch * 0.8; % J1: 좌우 회전
    q2 = -0.3 + abs(target_switch)*0.1; % J2: 살짝 끄덕임
    q3 = 0.6; % 팔 뻗기 유지
    
    angles = [q1; q2; q3; cos(t); -0.5; t*2; 0];

    for k = 2:length(links)
        m = makehgtform('translate', links(k).offset) * ...
            makehgtform('axisrotate', links(k).axis, angles(k-1));
        set(links(k).hg, 'Matrix', m);
    end
    
    drawnow;
    waitfor(rate);
end

%% Helper Function
function renderObjToGroup(parentHandle, obj, materials, materialAssignments)
    if isempty(materialAssignments)
        patch('Parent', parentHandle, 'Vertices', obj.v, 'Faces', obj.f.v, ...
              'FaceColor', [0.7 0.7 0.7], 'EdgeColor', 'none', 'FaceLighting', 'gouraud');
    else
        for i = 1:length(materialAssignments)
            matGroup = materialAssignments(i);
            if isempty(matGroup.faceIndices), continue; end
            faces = obj.f.v(matGroup.faceIndices, :);
            
            color = [0.7 0.7 0.7];
            if isfield(materials, matGroup.material)
                mat = materials.(matGroup.material);
                if isfield(mat, 'Kd'), color = mat.Kd; end
            end
            patch('Parent', parentHandle, 'Vertices', obj.v, 'Faces', faces, ...
                  'FaceColor', color, 'EdgeColor', 'none', 'FaceLighting', 'gouraud');
        end
    end
end