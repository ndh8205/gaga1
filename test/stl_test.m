%% 1. 환경 설정 및 데이터 정의
clear; clc; close all;

% STL 파일 경로
baseDir = 'D:\pj2025\space_challenge\model\modeling_3d\ASM_KARI_ARM\meshes';

% URDF 매핑 데이터
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

%% 2. 뷰어 초기화 (Grid OFF, 다크 모드)
f = figure('Color', 'k', 'Name', 'KARI ARM Clean View', ...
    'InvertHardcopy', 'off', ...
    'Renderer', 'opengl'); 

ax = axes('Parent', f, ...
    'Color', 'k', ...             % 배경 검은색
    'XColor', 'w', 'YColor', 'w', 'ZColor', 'w', ... % 축 선/글자 흰색
    'DataAspectRatio', [1 1 1], ... 
    'NextPlot', 'add'); 
    
% [핵심 변경] 그리드 끄기
grid off; 
hold on;
view(3);

% 축 라벨 (흰색 글씨)
xlabel('X'); ylabel('Y'); zlabel('Z');

% [흔들림 방지] 축 범위 고정
xlim([-1.2 1.2]);
ylim([-1.2 1.2]);
zlim([0 1.8]); 
axis vis3d manual; 

% 조명 설정
light('Position', [2 2 3], 'Style', 'local', 'Color', [1 1 1]);
light('Position', [-2 -2 1], 'Style', 'local', 'Color', [0.4 0.4 0.5]); 
lighting gouraud; 

links = struct('hg', [], 'axis', [], 'offset', []);

%% 3. 로봇 조립
for i = 1:size(robotDef, 1)
    fileName = robotDef{i,1};
    parentIdx = robotDef{i,2};
    offset = robotDef{i,3};
    rotAxis = robotDef{i,4};
    
    fullPath = fullfile(baseDir, fileName);
    if ~isfile(fullPath)
        error('File not found: %s', fullPath);
    end
    
    % 계층 구조
    if parentIdx == 0
        links(i).hg = hgtransform('Parent', ax);
    else
        links(i).hg = hgtransform('Parent', links(parentIdx).hg);
    end
    set(links(i).hg, 'Matrix', makehgtform('translate', offset));
    
    % STL 로드
    tr = stlread(fullPath);
    
    % 패치 생성
    p = patch('Parent', links(i).hg, ...
        'Faces', tr.ConnectivityList, ...
        'Vertices', tr.Points, ...
        'FaceColor', [0.9 0.9 0.92], ... % 쿨 화이트
        'EdgeColor', 'none', ...
        'FaceLighting', 'gouraud', ...
        'AmbientStrength', 0.3, ...
        'DiffuseStrength', 0.6, ...
        'SpecularStrength', 0.8, ...
        'SpecularExponent', 15);
    material(p, 'dull'); 
    
    links(i).axis = rotAxis;
    links(i).offset = offset;
end

% 초기 시점
view(45, 30); 
% axis off; % (선택사항) X,Y,Z 축 선조차 없애고 싶으면 이 주석을 해제하세요.

%% 4. 애니메이션 루프
disp('애니메이션 시작... (Grid 제거됨)');
t = 0;
rate = rateControl(30); 

while ishandle(f)
    t = t + 0.02; 
    
    angles = [
        sin(t) * 0.5;
        sin(t*0.7) * 0.3;
        sin(t) * 0.4;
        cos(t) * 0.4;
        0;
        sin(t) * 0.4;
        t * 1.5
    ];

    for k = 2:length(links)
        m = makehgtform('translate', links(k).offset) * ...
            makehgtform('axisrotate', links(k).axis, angles(k-1));
        set(links(k).hg, 'Matrix', m);
    end
    
    drawnow;
    waitfor(rate);
end