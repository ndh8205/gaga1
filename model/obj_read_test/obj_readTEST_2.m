%% OBJ 파일 시각화 테스트 (최적화 버전)
clear; close all; clc;

%% 2. 6U CubeSat 시각화
fprintf('Loading 6U CubeSat...\n');
tic;
visualizeObjWithColor('6U_CubeSat_v7_nopayload/6U_CubeSat_v7_nopayload.obj');
fprintf('Loading time: %.2f seconds\n', toc);

%% 3. 3U CNU CubeSat 시각화
fprintf('\nLoading 3U CNU CubeSat...\n');
tic;
visualizeObjWithColor('1015_3U_CNU/1015_3U_CNU.obj');
fprintf('Loading time: %.2f seconds\n', toc);

%% 4. 두 모델 비교 (검은 배경)
figure('Name', 'CubeSat Comparison', 'Position', [100 100 1200 500], ...
       'Color', 'k');  % Figure 배경 검은색

% 6U CubeSat
subplot(1,2,1);
tic;
[obj6U, mat6U, matAssign6U] = readObjFast('6U_CubeSat_v7_nopayload/6U_CubeSat_v7_nopayload.obj');
fprintf('6U loaded in %.2f sec\n', toc);
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
tic;
[obj3U, mat3U, matAssign3U] = readObjFast('1015_3U_CNU/1015_3U_CNU.obj');
fprintf('3U loaded in %.2f sec\n', toc);
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

%% ========== 최적화된 Fast 함수들 ==========

function [obj, materials, materialAssignments] = readObjFast(filename)
    % 빠른 OBJ 파서 - 최적화 버전
    
    % 초기화
    obj = struct('v', [], 'vt', [], 'vn', [], 'f', struct('v', [], 'vt', [], 'vn', []));
    materials = struct();
    materialAssignments = struct('material', {}, 'faceIndices', {});
    
    % 파일 전체를 한 번에 읽기
    fileContent = fileread(filename);
    lines = strsplit(fileContent, '\n');
    
    % 미리 카운트하여 메모리 할당
    nVertices = sum(startsWith(lines, 'v '));
    nFaces = sum(startsWith(lines, 'f '));
    nNormals = sum(startsWith(lines, 'vn '));
    nTexCoords = sum(startsWith(lines, 'vt '));
    
    % 메모리 미리 할당
    obj.v = zeros(nVertices, 3);
    obj.vn = zeros(nNormals, 3);
    obj.vt = zeros(nTexCoords, 2);
    obj.f.v = zeros(nFaces * 2, 3);  % 사각형 고려하여 여유 할당
    
    % 카운터
    vIdx = 0; vtIdx = 0; vnIdx = 0; fIdx = 0;
    currentMaterial = '';
    currentMatIndex = 0;
    
    % MTL 파일 먼저 찾아서 로드
    for i = 1:length(lines)
        if startsWith(lines{i}, 'mtllib')
            parts = strsplit(lines{i});
            if length(parts) >= 2
                mtlFile = strjoin(parts(2:end), ' ');
                [path, ~, ~] = fileparts(filename);
                mtlPath = fullfile(path, mtlFile);
                if exist(mtlPath, 'file')
                    materials = readMtlFast(mtlPath);
                end
                break;
            end
        end
    end
    
    % 빠른 파싱
    for i = 1:length(lines)
        line = strtrim(lines{i});
        if isempty(line) || line(1) == '#'
            continue;
        end
        
        % 첫 2글자로 빠른 판별
        if length(line) >= 2
            prefix = line(1:2);
            
            switch prefix
                case 'v '
                    % Vertex - sscanf가 str2double보다 빠름
                    vIdx = vIdx + 1;
                    obj.v(vIdx, :) = sscanf(line(3:end), '%f %f %f')';
                    
                case 'vn'
                    vnIdx = vnIdx + 1;
                    obj.vn(vnIdx, :) = sscanf(line(4:end), '%f %f %f')';
                    
                case 'vt'
                    vtIdx = vtIdx + 1;
                    data = sscanf(line(4:end), '%f %f');
                    obj.vt(vtIdx, 1:length(data)) = data';
                    
                case 'f '
                    % Face - 빠른 처리
                    faceData = strsplit(line(3:end));
                    nVerts = length(faceData);
                    
                    if nVerts == 3
                        % 삼각형
                        fIdx = fIdx + 1;
                        for j = 1:3
                            indices = sscanf(faceData{j}, '%d/%d/%d');
                            obj.f.v(fIdx, j) = indices(1);
                        end
                    elseif nVerts == 4
                        % 사각형을 두 개의 삼각형으로
                        verts = zeros(1, 4);
                        for j = 1:4
                            indices = sscanf(faceData{j}, '%d/%d/%d');
                            verts(j) = indices(1);
                        end
                        fIdx = fIdx + 1;
                        obj.f.v(fIdx, :) = verts([1 2 3]);
                        fIdx = fIdx + 1;
                        obj.f.v(fIdx, :) = verts([1 3 4]);
                    end
                    
                    % Material assignment
                    if currentMatIndex > 0
                        materialAssignments(currentMatIndex).faceIndices(end+1) = fIdx;
                    end
                    
                case 'us'  % usemtl
                    if startsWith(line, 'usemtl')
                        parts = strsplit(line);
                        if length(parts) >= 2
                            currentMaterial = parts{2};
                            currentMatIndex = currentMatIndex + 1;
                            materialAssignments(currentMatIndex).material = currentMaterial;
                            materialAssignments(currentMatIndex).faceIndices = [];
                        end
                    end
            end
        end
    end
    
    % 사용하지 않은 메모리 제거
    obj.v = obj.v(1:vIdx, :);
    obj.vn = obj.vn(1:vnIdx, :);
    obj.vt = obj.vt(1:vtIdx, :);
    obj.f.v = obj.f.v(1:fIdx, :);
end

function materials = readMtlFast(filename)
    % 빠른 MTL 파서
    materials = struct();
    
    % 파일 전체 읽기
    fileContent = fileread(filename);
    lines = strsplit(fileContent, '\n');
    
    currentMat = '';
    
    for i = 1:length(lines)
        line = strtrim(lines{i});
        if isempty(line) || line(1) == '#'
            continue;
        end
        
        % 빠른 명령어 체크
        if startsWith(line, 'newmtl ')
            currentMat = strrep(line(8:end), ' ', '_');
            currentMat = matlab.lang.makeValidName(currentMat);
            materials.(currentMat) = struct();
            
        elseif ~isempty(currentMat)
            if startsWith(line, 'Kd ')
                % Diffuse color
                materials.(currentMat).Kd = sscanf(line(4:end), '%f %f %f')';
                
            elseif startsWith(line, 'Ka ')
                % Ambient color
                materials.(currentMat).Ka = sscanf(line(4:end), '%f %f %f')';
                
            elseif startsWith(line, 'Ks ')
                % Specular color
                materials.(currentMat).Ks = sscanf(line(4:end), '%f %f %f')';
                
            elseif startsWith(line, 'Ns ')
                % Shininess
                materials.(currentMat).Ns = sscanf(line(4:end), '%f');
                
            elseif startsWith(line, 'd ')
                % Transparency
                materials.(currentMat).d = sscanf(line(3:end), '%f');
            end
        end
    end
end

function visualizeObjWithColor(objFilePath)
    % 메인 시각화 함수 - 최적화 버전
    [obj, materials, materialAssignments] = readObjFast(objFilePath);
    
    % 시각화
    figure('Name', 'OBJ Viewer', 'Color', 'w');
    hold on;
    
    % Material이 없으면 단일 patch로 빠르게 그리기
    if isempty(materialAssignments)
        patch('Vertices', obj.v, ...
              'Faces', obj.f.v, ...
              'FaceColor', [0.7 0.7 0.7], ...
              'EdgeColor', 'none', ...
              'FaceLighting', 'gouraud');
    else
        % Material별로 그리기 (최적화)
        for i = 1:length(materialAssignments)
            matGroup = materialAssignments(i);
            if isempty(matGroup.faceIndices)
                continue;
            end
            
            faces = obj.f.v(matGroup.faceIndices, :);
            
            % 색상 결정
            color = [0.7 0.7 0.7];  % 기본값
            if isfield(materials, matGroup.material)
                mat = materials.(matGroup.material);
                if isfield(mat, 'Kd')
                    color = mat.Kd;
                end
            end
            
            % 한 번에 그리기
            patch('Vertices', obj.v, ...
                  'Faces', faces, ...
                  'FaceColor', color, ...
                  'EdgeColor', 'none', ...
                  'FaceLighting', 'gouraud');
        end
    end
    
    % 빠른 설정
    axis equal;
    grid on;
    xlabel('X'); ylabel('Y'); zlabel('Z');
    camlight('headlight');
    lighting gouraud;
    view(45, 30);
    rotate3d on;
    
    [~, name, ~] = fileparts(objFilePath);
    title(sprintf('Model: %s', name));
end

function plotObjModel(obj, materials, materialAssignments)
    hold on;
    
    % Material이 없는 경우 - 빠른 단일 렌더링
    if isempty(materialAssignments)
        patch('Vertices', obj.v, ...
              'Faces', obj.f.v, ...
              'FaceColor', [0.7 0.7 0.7], ...
              'EdgeColor', 'none', ...
              'FaceLighting', 'gouraud');
    else
        % Material별 그리기 - 색상 미리 계산
        colors = cell(length(materialAssignments), 1);
        faceGroups = cell(length(materialAssignments), 1);
        
        for i = 1:length(materialAssignments)
            matGroup = materialAssignments(i);
            if isempty(matGroup.faceIndices)
                continue;
            end
            
            faceGroups{i} = obj.f.v(matGroup.faceIndices, :);
            
            % 색상 결정 (한 번만)
            if isfield(materials, matGroup.material) && isfield(materials.(matGroup.material), 'Kd')
                colors{i} = materials.(matGroup.material).Kd;
            else
                % 기본 색상 매핑
                matName = lower(matGroup.material);
                if contains(matName, 'solar') || contains(matName, 'panel')
                    colors{i} = [0.1 0.1 0.8];
                elseif contains(matName, 'antenna')
                    colors{i} = [0.9 0.9 0.1];
                else
                    colors{i} = [0.7 0.7 0.7];
                end
            end
        end
        
        % 한 번에 그리기
        for i = 1:length(faceGroups)
            if ~isempty(faceGroups{i})
                patch('Vertices', obj.v, ...
                      'Faces', faceGroups{i}, ...
                      'FaceColor', colors{i}, ...
                      'EdgeColor', 'none', ...
                      'FaceLighting', 'gouraud');
            end
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
    fprintf('  Center: [%.2f, %.2f, %.2f]\n', mean([minV; maxV]));
    fprintf('  Vertices: %d\n', size(vertices, 1));
end