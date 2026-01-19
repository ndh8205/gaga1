function clearCache(objFilePath)
    % 특정 파일의 캐시 삭제
    [path, name, ~] = fileparts(objFilePath);
    cacheDir = fullfile(path, '.cache');
    cacheFile = fullfile(cacheDir, [name '_cache.mat']);
    
    if exist(cacheFile, 'file')
        delete(cacheFile);
        fprintf('Cache cleared for %s\n', name);
    end
end