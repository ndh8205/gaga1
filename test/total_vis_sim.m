%% 통합 시뮬레이션: KARI 로봇 팔 & 6U 큐브 위성
clear; clc; close all;

% [1] 경로 설정 (사용자 환경에 맞춤)
baseDir = 'D:\pj2025\space_challenge';
addpath(genpath(baseDir)); % 모든 하위 폴더 경로 추가

% 파일 경로 정의
robotMeshPath = fullfile(baseDir, 'model', 'modeling_3d', 'ASM_KARI_ARM', 'meshes');
satObjPath = fullfile(baseDir, 'model', 'modeling_3d', '6U_CubeSat_v7_nopayload', '6U_CubeSat_v7_nopayload.obj');

%% [2] 뷰어 및 환경 초기화 (Dark Space Mode)
f = figure('Color', 'k', 'Name', 'Space Operation Simulation', ...
    'InvertHardcopy', 'off', 'Renderer', 'opengl', ...
    'Position', [100 100 1200 800]);

ax = axes('Parent', f, 'Color', 'k', ...
    'XColor', 'w', 'YColor', 'w', 'ZColor', 'w', ...
    'DataAspectRatio', [1 1 1], 'NextPlot', 'add');

grid off; axis off; % 그리드와 축 제거 (완전한 우주 공간 느낌)
view(45, 20);
xlim([-2.0 2.0]); ylim([-2.0 2.0]); zlim([0 2.5]);
axis vis3d manual; % 줌인/아웃 시 배경 울렁거림 방지

% 조명 설정 (태양광 느낌의 강한 대비)
light('Position', [3 1 3], 'Style', 'local', 'Color', [1 1 0.9]); % 주 광원 (Warm white)
light('Position', [-2 -2 0.5], 'Style', 'local', 'Color', [0.1 0.2 0.4]); % 반사광 (Cool blue)
lighting gouraud;

%% [3] 6U CubeSat 로드 및 배치
fprintf('Loading CubeSat...\n');
if ~isfile(satObjPath)
    error('CubeSat OBJ 파일을 찾을 수 없습니다: %s', satObjPath);
end

% 사용자 함수 이용해 OBJ 데이터 로드
[satData, satMats, satAssign] = readObjWithCache(satObjPath);

% 위성용 Transform 생성 (위치를 이동하거나 회전시키기 위함)
satGroup = hgtransform('Parent', ax);

% *** 중요: 스케일 및 초기 위치 조정 ***
% OBJ가 mm단위라면 0.001을 곱해야 m단위 로봇과 맞습니다.
% 너무 크거나 작으면 이 scaleFactor를 조절하세요.
scaleFactor = 0.001; 
satPos = [0.6, 0.0, 0.5]; % 로봇 앞쪽 60cm, 높이 50cm 지점에 배치

% 위성 렌더링 (아래 작성한 헬퍼 함수 사용)
renderObjToGroup(satGroup, satData, satMats, satAssign);

% 초기 위치 설정 (Scale -> Rotate -> Translate)
set(satGroup, 'Matrix', makehgtform('translate', satPos) * makehgtform('scale', scaleFactor));


%% [4] 로봇 팔(ASM_KARI_ARM) 로드 및 조립
fprintf('Loading Robot Arm...\n');

% URDF 정보
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
    fileName = robotDef{i,1};
    parentIdx = robotDef{i,2};
    offset = robotDef{i,3};
    rotAxis = robotDef{i,4};
    
    fullPath = fullfile(robotMeshPath, fileName);
    if ~isfile(fullPath)
        warning('Mesh not found: %s', fullPath); continue;
    end
    
    % 계층 구조 생성
    if parentIdx == 0
        links(i).hg = hgtransform('Parent', ax);
    else
        links(i).hg = hgtransform('Parent', links(parentIdx).hg);
    end
    set(links(i).hg, 'Matrix', makehgtform('translate', offset));
    
    % STL 로드
    tr = stlread(fullPath);
    
    % 로봇 렌더링 (Clean White)
    p = patch('Parent', links(i).hg, ...
        'Faces', tr.ConnectivityList, 'Vertices', tr.Points, ...
        'FaceColor', [0.9 0.9 0.92], 'EdgeColor', 'none', ...
        'FaceLighting', 'gouraud', 'AmbientStrength', 0.3, ...
        'DiffuseStrength', 0.6, 'SpecularStrength', 0.8, 'SpecularExponent', 20);
    material(p, 'dull'); 
    
    links(i).axis = rotAxis;
    links(i).offset = offset;
end

%% [5] 시네마틱 애니메이션 루프
disp('Simulation Started. Press Ctrl+C to stop.');

t = 0;
rate = rateControl(60); % 60 FPS 부드러운 움직임

while ishandle(f)
    t = t + 0.01;
    
    % --- 1. 위성 움직임 (자전 + 부유) ---
    % 제자리에서 천천히 회전 (Tumbling)
    satRot = makehgtform('axisrotate', [1 1 0], t*0.5); 
    % 약간 위아래로 둥둥 떠다니는 느낌 (Floating)
    floatOffset = [0, 0, sin(t)*0.05]; 
    
    % 위성 매트릭스 업데이트
    set(satGroup, 'Matrix', makehgtform('translate', satPos + floatOffset) * satRot * makehgtform('scale', scaleFactor));
    
    % --- 2. 로봇 팔 움직임 (위성 추적 느낌) ---
    % 복잡한 역기구학 대신, 시나리오에 맞는 동작 패턴 생성
    
    % J1 (Base): 위성 방향을 향해 좌우로 살짝 스캔
    q1 = sin(t*0.5) * 0.2; 
    
    % J2 (Shoulder): 팔을 들어올림
    q2 = -0.2 + sin(t*0.3)*0.1;
    
    % J3 (Elbow): 팔을 뻗음
    q3 = 0.5 + cos(t*0.3)*0.1;
    
    % J4~J6 (Wrist): 끝단이 위성을 바라보도록 조정
    q4 = 0; 
    q5 = -0.3; % 손목 꺾임
    q6 = t * 2; % 끝단 회전 (작업 중인 것처럼)
    q7 = 0;
    
    angles = [q1; q2; q3; q4; q5; q6; q7];

    % 로봇 관절 업데이트
    for k = 2:length(links) % Base(idx=1)은 고정, idx=2(J1)부터 움직임
        m = makehgtform('translate', links(k).offset) * ...
            makehgtform('axisrotate', links(k).axis, angles(k-1));
        set(links(k).hg, 'Matrix', m);
    end
    
    drawnow;
    waitfor(rate);
end


%% [Helper Function] OBJ 렌더링을 hgtransform 그룹에 적용
function renderObjToGroup(parentHandle, obj, materials, materialAssignments)
    % plotObjModel.m의 로직을 가져와서 Parent를 지정할 수 있게 수정함
    
    if isempty(materialAssignments)
        % Material 없음
        patch('Parent', parentHandle, ...
              'Vertices', obj.v, 'Faces', obj.f.v, ...
              'FaceColor', [0.5 0.5 0.5], 'EdgeColor', 'none', ...
              'FaceLighting', 'gouraud');
    else
        % Material별 그리기
        for i = 1:length(materialAssignments)
            matGroup = materialAssignments(i);
            if isempty(matGroup.faceIndices), continue; end
            
            faces = obj.f.v(matGroup.faceIndices, :);
            
            % 색상 추출
            color = [0.7 0.7 0.7]; % Default
            if isfield(materials, matGroup.material)
                mat = materials.(matGroup.material);
                if isfield(mat, 'Kd'), color = mat.Kd; end
            else
                % 이름 기반 색상 추정 (Fallback)
                mName = lower(matGroup.material);
                if contains(mName, 'solar'), color = [0.1 0.1 0.5];
                elseif contains(mName, 'gold'), color = [0.8 0.6 0.1];
                end
            end
            
            patch('Parent', parentHandle, ...
                  'Vertices', obj.v, 'Faces', faces, ...
                  'FaceColor', color, 'EdgeColor', 'none', ...
                  'FaceLighting', 'gouraud', ...
                  'SpecularStrength', 0.5); % 위성 반사광
        end
    end
end