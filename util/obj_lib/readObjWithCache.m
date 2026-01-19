function [obj, materials, materialAssignments] = readObjWithCache(objFilePath)
    % 캐싱을 사용한 OBJ 로더
    
    % 캐시 파일 경로 생성
    [path, name, ~] = fileparts(objFilePath);
    cacheDir = fullfile(path, '.cache');
    cacheFile = fullfile(cacheDir, [name '_cache.mat']);
    
    % 캐시 디렉토리 생성
    if ~exist(cacheDir, 'dir')
        mkdir(cacheDir);
    end
    
    % 원본 파일 정보
    objInfo = dir(objFilePath);
    objDate = objInfo.datenum;
    objSize = objInfo.bytes;
    
    % MTL 파일 정보도 체크
    mtlFilePath = strrep(objFilePath, '.obj', '.mtl');
    mtlDate = 0;
    mtlSize = 0;
    if exist(mtlFilePath, 'file')
        mtlInfo = dir(mtlFilePath);
        mtlDate = mtlInfo.datenum;
        mtlSize = mtlInfo.bytes;
    end
    
    % 캐시 파일 확인
    needsUpdate = true;
    if exist(cacheFile, 'file')
        try
            % 캐시 로드
            cache = load(cacheFile);
            
            % 버전 및 날짜 확인
            if isfield(cache, 'version') && cache.version == 1.0 && ...
               isfield(cache, 'objDate') && cache.objDate == objDate && ...
               isfield(cache, 'objSize') && cache.objSize == objSize && ...
               isfield(cache, 'mtlDate') && cache.mtlDate == mtlDate && ...
               isfield(cache, 'mtlSize') && cache.mtlSize == mtlSize
                
                % 캐시가 유효함
                obj = cache.obj;
                materials = cache.materials;
                materialAssignments = cache.materialAssignments;
                needsUpdate = false;
                fprintf('  [Using cached data from %s]\n', cacheFile);
            end
        catch
            % 캐시 로드 실패
            fprintf('  [Cache corrupted, rebuilding...]\n');
        end
    end
    
    % 캐시 업데이트가 필요한 경우
    if needsUpdate
        fprintf('  [Parsing OBJ file...]\n');
        [obj, materials, materialAssignments] = readObjFast(objFilePath);
        
        % 캐시 저장
        fprintf('  [Saving cache...]\n');
        version = 1.0;
        save(cacheFile, 'obj', 'materials', 'materialAssignments', ...
             'objDate', 'objSize', 'mtlDate', 'mtlSize', 'version', '-v7.3');
        fprintf('  [Cache saved to %s]\n', cacheFile);
    end
end