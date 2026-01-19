%% OBJ 파일 시각화 테스트
clear; close all; clc;

%% 2. 6U CubeSat 시각화
fprintf('Loading 6U CubeSat...\n');
visualizeObjWithColor('6U_CubeSat_v7_nopayload/6U_CubeSat_v7_nopayload.obj');

%% 3. 3U CNU CubeSat 시각화
fprintf('\nLoading 3U CNU CubeSat...\n');
visualizeObjWithColor('1015_3U_CNU/1015_3U_CNU.obj');

% %% 4. 두 모델 비교
% figure('Name', 'CubeSat Comparison', 'Position', [100 100 1200 500]);
% 
% % 6U CubeSat
% subplot(1,2,1);
% [obj6U, mat6U, matAssign6U] = readObjComplete('6U_CubeSat_v7_nopayload/6U_CubeSat_v7_nopayload.obj');
% plotObjModel(obj6U, mat6U, matAssign6U);
% title('6U CubeSat');
% view(45, 30);
% 
% % 3U CubeSat
% subplot(1,2,2);
% [obj3U, mat3U, matAssign3U] = readObjComplete('1015_3U_CNU/1015_3U_CNU.obj');
% plotObjModel(obj3U, mat3U, matAssign3U);
% title('3U CNU CubeSat');
% view(45, 30);

%% 4. 두 모델 비교 (검은 배경)
figure('Name', 'CubeSat Comparison', 'Position', [100 100 1200 500], ...
       'Color', 'k');  % Figure 배경 검은색

% 6U CubeSat
subplot(1,2,1);
[obj6U, mat6U, matAssign6U] = readObjComplete('6U_CubeSat_v7_nopayload/6U_CubeSat_v7_nopayload.obj');
plotObjModel(obj6U, mat6U, matAssign6U);
title('6U CubeSat', 'Color', 'w');  % 제목 글자 흰색
view(45, 30);

% Axes 배경 검은색, 그리드와 라벨 흰색
ax1 = gca;
ax1.Color = 'k';
ax1.XColor = 'w';
ax1.YColor = 'w';
ax1.ZColor = 'w';
ax1.GridColor = [0.3 0.3 0.3];  % 그리드 어두운 회색
ax1.GridAlpha = 0.3;

% 3U CubeSat
subplot(1,2,2);
[obj3U, mat3U, matAssign3U] = readObjComplete('1015_3U_CNU/1015_3U_CNU.obj');
plotObjModel(obj3U, mat3U, matAssign3U);
title('3U CNU CubeSat', 'Color', 'w');  % 제목 글자 흰색
view(45, 30);

% Axes 배경 검은색, 그리드와 라벨 흰색
ax2 = gca;
ax2.Color = 'k';
ax2.XColor = 'w';
ax2.YColor = 'w';
ax2.ZColor = 'w';
ax2.GridColor = [0.3 0.3 0.3];  % 그리드 어두운 회색
ax2.GridAlpha = 0.3;

%% 5. 크기 비교
fprintf('\n=== Model Size Comparison ===\n');
fprintf('6U CubeSat:\n');
printModelDimensions(obj6U.v);
fprintf('\n3U CubeSat:\n');
printModelDimensions(obj3U.v);

%% Helper Functions
function plotObjModel(obj, materials, materialAssignments)
    hold on;
    
    % Material이 없는 경우
    if isempty(materialAssignments)
        patch('Vertices', obj.v, ...
              'Faces', obj.f.v, ...
              'FaceColor', [0.7 0.7 0.7], ...
              'EdgeColor', 'none', ...
              'FaceLighting', 'gouraud');
    else
        % Material별로 그리기
        for i = 1:length(materialAssignments)
            matGroup = materialAssignments(i);
            
            if isempty(matGroup.faceIndices)
                continue;
            end
            
            faces = obj.f.v(matGroup.faceIndices, :);
            
            % Material 색상
            if isfield(materials, matGroup.material)
                mat = materials.(matGroup.material);
                if isfield(mat, 'Kd')
                    color = mat.Kd;
                else
                    color = [0.7 0.7 0.7];
                end
            else
                % Material 이름으로 색상 추정
                if contains(lower(matGroup.material), 'solar') || ...
                   contains(lower(matGroup.material), 'panel')
                    color = [0.1 0.1 0.8];  % 파란색 (태양 패널)
                elseif contains(lower(matGroup.material), 'antenna')
                    color = [0.9 0.9 0.1];  % 노란색 (안테나)
                elseif contains(lower(matGroup.material), 'body') || ...
                       contains(lower(matGroup.material), 'frame')
                    color = [0.7 0.7 0.7];  % 회색 (본체)
                else
                    color = [0.5 0.5 0.5];  % 기본 회색
                end
            end
            
            patch('Vertices', obj.v, ...
                  'Faces', faces, ...
                  'FaceColor', color, ...
                  'EdgeColor', 'none', ...
                  'FaceLighting', 'gouraud');
        end
    end
    
    axis equal;
    grid on;
    xlabel('X'); ylabel('Y'); zlabel('Z');
    camlight('headlight');
    lighting gouraud;
    material shiny;
    rotate3d on;
end

function printModelDimensions(vertices)
    minV = min(vertices);
    maxV = max(vertices);
    dims = maxV - minV;
    
    fprintf('  Dimensions (X × Y × Z): %.2f × %.2f × %.2f units\n', ...
            dims(1), dims(2), dims(3));
    fprintf('  Center: [%.2f, %.2f, %.2f]\n', ...
            mean([minV; maxV]));
    fprintf('  Vertices: %d\n', size(vertices, 1));
end

function visualizeObjWithColor(objFilePath)
    % 메인 함수 - OBJ 파일을 색상과 함께 시각화
    [obj, materials, materialAssignments] = readObjComplete(objFilePath);
    
    % 시각화
    figure('Name', 'OBJ Viewer with Materials', 'Color', 'w');
    hold on;
    
    % Material별로 그리기
    for i = 1:length(materialAssignments)
        matGroup = materialAssignments(i);
        
        % 해당 material의 face들만 추출
        faces = obj.f.v(matGroup.faceIndices, :);
        
        % Material 색상 가져오기
        if isfield(materials, matGroup.material)
            mat = materials.(matGroup.material);
            if isfield(mat, 'Kd')
                color = mat.Kd;
            elseif isfield(mat, 'Ka')
                color = mat.Ka;
            else
                color = [0.7 0.7 0.7];  % 기본 회색
            end
        else
            color = [0.7 0.7 0.7];
        end
        
        % Patch 그리기
        h = patch('Vertices', obj.v, ...
                  'Faces', faces, ...
                  'FaceColor', color, ...
                  'EdgeColor', 'none', ...
                  'FaceLighting', 'gouraud');
        
        % Material 속성 적용
        if isfield(materials, matGroup.material)
            mat = materials.(matGroup.material);
            if isfield(mat, 'Ks') && isfield(mat, 'Ns')
                % Specular 설정
                material(h, [0.3, 0.6, mat.Ks(1), mat.Ns/100]);
            end
            if isfield(mat, 'd')
                % 투명도 설정
                set(h, 'FaceAlpha', mat.d);
            end
        end
    end
    
    % 조명 및 카메라 설정
    axis equal;
    grid on;
    xlabel('X'); ylabel('Y'); zlabel('Z');
    
    camlight('headlight');
    lighting gouraud;
    view(45, 30);
    rotate3d on;
    
    title(sprintf('Model: %s', objFilePath));
end

function [obj, materials, materialAssignments] = readObjComplete(filename)
    % 완전한 OBJ 파서 (MTL 지원 포함)
    
    % 초기화
    obj.v = [];   % vertices
    obj.vt = [];  % texture coords
    obj.vn = [];  % normals
    obj.f.v = []; % face vertices
    obj.f.vt = []; % face texture
    obj.f.vn = []; % face normals
    
    materials = struct();
    materialAssignments = struct('material', {}, 'faceIndices', {});
    currentMaterial = '';
    currentMatIndex = 0;
    faceCounter = 0;
    
    % OBJ 파일 열기
    fid = fopen(filename, 'r');
    if fid == -1
        error('Cannot open file: %s', filename);
    end
    
    % OBJ 파일 파싱
    while ~feof(fid)
        line = fgetl(fid);
        if ~ischar(line), continue; end
        
        % 빈 줄 무시
        line = strtrim(line);
        if isempty(line) || line(1) == '#', continue; end
        
        parts = strsplit(line);
        cmd = parts{1};
        
        switch cmd
            case 'mtllib'
                % MTL 파일 읽기
                mtlFile = strjoin(parts(2:end), ' ');
                [path, ~, ~] = fileparts(filename);
                mtlPath = fullfile(path, mtlFile);
                if exist(mtlPath, 'file')
                    materials = readMtlFile(mtlPath);
                    fprintf('Loaded material library: %s\n', mtlFile);
                end
                
            case 'usemtl'
                % Material 변경
                currentMaterial = parts{2};
                currentMatIndex = currentMatIndex + 1;
                materialAssignments(currentMatIndex).material = currentMaterial;
                materialAssignments(currentMatIndex).faceIndices = [];
                
            case 'v'
                % Vertex
                obj.v(end+1, :) = [str2double(parts{2}), ...
                                    str2double(parts{3}), ...
                                    str2double(parts{4})];
                
            case 'vt'
                % Texture coordinate
                obj.vt(end+1, :) = [str2double(parts{2}), ...
                                     str2double(parts{3})];
                
            case 'vn'
                % Normal
                obj.vn(end+1, :) = [str2double(parts{2}), ...
                                     str2double(parts{3}), ...
                                     str2double(parts{4})];
                
            case 'f'
                % Face
                faceCounter = faceCounter + 1;
                face_v = [];
                face_vt = [];
                face_vn = [];
                
                for i = 2:length(parts)
                    indices = strsplit(parts{i}, '/');
                    
                    % Vertex index
                    face_v(end+1) = str2double(indices{1});
                    
                    % Texture index (optional)
                    if length(indices) > 1 && ~isempty(indices{2})
                        face_vt(end+1) = str2double(indices{2});
                    end
                    
                    % Normal index (optional)
                    if length(indices) > 2 && ~isempty(indices{3})
                        face_vn(end+1) = str2double(indices{3});
                    end
                end
                
                % Face 저장
                if length(face_v) == 3
                    % 삼각형
                    obj.f.v(end+1, :) = face_v;
                elseif length(face_v) == 4
                    % 사각형을 삼각형 2개로 분할
                    obj.f.v(end+1, :) = face_v([1 2 3]);
                    obj.f.v(end+1, :) = face_v([1 3 4]);
                    faceCounter = faceCounter + 1;  % 추가 face
                else
                    % 다각형 - 팬 삼각화
                    for i = 2:length(face_v)-1
                        obj.f.v(end+1, :) = face_v([1 i i+1]);
                        if i > 2
                            faceCounter = faceCounter + 1;
                        end
                    end
                end
                
                % Material assignment
                if currentMatIndex > 0
                    materialAssignments(currentMatIndex).faceIndices(end+1) = ...
                        size(obj.f.v, 1);
                end
        end
    end
    
    fclose(fid);
    
    fprintf('Loaded: %d vertices, %d faces, %d materials\n', ...
            size(obj.v, 1), size(obj.f.v, 1), length(fieldnames(materials)));
end

function materials = readMtlFile(filename)
    % MTL 파일 파서
    materials = struct();
    currentMat = '';
    
    fid = fopen(filename, 'r');
    if fid == -1
        warning('Cannot open MTL file: %s', filename);
        return;
    end
    
    while ~feof(fid)
        line = fgetl(fid);
        if ~ischar(line), continue; end
        
        line = strtrim(line);
        if isempty(line) || line(1) == '#', continue; end
        
        parts = strsplit(line);
        cmd = parts{1};
        
        switch cmd
            case 'newmtl'
                % 새 material 정의
                currentMat = strjoin(parts(2:end), '_');
                % MATLAB 구조체 필드명으로 사용할 수 있게 변경
                currentMat = matlab.lang.makeValidName(currentMat);
                materials.(currentMat) = struct();
                
            case 'Ka'
                % Ambient color
                if ~isempty(currentMat)
                    materials.(currentMat).Ka = [str2double(parts{2}), ...
                                                  str2double(parts{3}), ...
                                                  str2double(parts{4})];
                end
                
            case 'Kd'
                % Diffuse color (주 색상)
                if ~isempty(currentMat)
                    materials.(currentMat).Kd = [str2double(parts{2}), ...
                                                  str2double(parts{3}), ...
                                                  str2double(parts{4})];
                end
                
            case 'Ks'
                % Specular color
                if ~isempty(currentMat)
                    materials.(currentMat).Ks = [str2double(parts{2}), ...
                                                  str2double(parts{3}), ...
                                                  str2double(parts{4})];
                end
                
            case 'Ns'
                % Specular exponent (광택도)
                if ~isempty(currentMat)
                    materials.(currentMat).Ns = str2double(parts{2});
                end
                
            case 'd'
                % 투명도 (1=불투명, 0=투명)
                if ~isempty(currentMat)
                    materials.(currentMat).d = str2double(parts{2});
                end
                
            case 'Tr'
                % 투명도 (역방향, 0=불투명, 1=투명)
                if ~isempty(currentMat)
                    materials.(currentMat).d = 1 - str2double(parts{2});
                end
                
            case 'illum'
                % 조명 모델
                if ~isempty(currentMat)
                    materials.(currentMat).illum = str2double(parts{2});
                end
                
            case 'map_Kd'
                % Diffuse texture map
                if ~isempty(currentMat)
                    materials.(currentMat).map_Kd = strjoin(parts(2:end), ' ');
                end
        end
    end
    
    fclose(fid);
    
    % Material 정보 출력
    matNames = fieldnames(materials);
    for i = 1:length(matNames)
        mat = materials.(matNames{i});
        fprintf('Material "%s":\n', matNames{i});
        if isfield(mat, 'Kd')
            fprintf('  Diffuse: [%.2f, %.2f, %.2f]\n', mat.Kd);
        end
        if isfield(mat, 'Ks')
            fprintf('  Specular: [%.2f, %.2f, %.2f]\n', mat.Ks);
        end
    end
end


function applyTextureIfExists(h, material, basePath)
    % 텍스처 파일이 있으면 적용
    if isfield(material, 'map_Kd')
        texturePath = fullfile(basePath, material.map_Kd);
        if exist(texturePath, 'file')
            img = imread(texturePath);
            set(h, 'FaceColor', 'texturemap', ...
                   'CData', img);
            fprintf('Applied texture: %s\n', material.map_Kd);
        end
    end
end