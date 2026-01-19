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
                        
                        % Material assignment
                        if currentMatIndex > 0
                            materialAssignments(currentMatIndex).faceIndices(end+1) = fIdx;
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
                        if currentMatIndex > 0
                            materialAssignments(currentMatIndex).faceIndices(end+1) = fIdx;
                        end
                        fIdx = fIdx + 1;
                        obj.f.v(fIdx, :) = verts([1 3 4]);
                        if currentMatIndex > 0
                            materialAssignments(currentMatIndex).faceIndices(end+1) = fIdx;
                        end
                    end
                    
                case 'us'  % usemtl
                    if startsWith(line, 'usemtl')
                        parts = strsplit(line);
                        if length(parts) >= 2
                            % MTL과 동일하게 변환
                            rawName = strrep(parts{2}, ' ', '_');
                            currentMaterial = matlab.lang.makeValidName(rawName);
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
    
    fprintf('    Parsed: %d vertices, %d faces, %d materials\n', ...
            size(obj.v, 1), size(obj.f.v, 1), length(materialAssignments));
end