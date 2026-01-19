clc; clear all; close all;
addpath(genpath('D:\star_tracker_test\main_pj_code')); % 쿼터니언 라이브러리 경로

% STL 파일 읽기
% filename = 'cube_sat.stl'; % STL 파일 경로를 여기에 입력
% filename = 'chief.stl'; % STL 파일 경로를 여기에 입력
filename = 'ksas.stl'; % STL 파일 경로를 여기에 입력

TR = stlread(filename);

% STL 파일이 제대로 불러와졌는지 확인 및 정보 출력
fprintf('=== STL 파일 정보 ===\n');
fprintf('삼각형 면의 개수: %d\n', size(TR.ConnectivityList, 1));
fprintf('정점의 개수: %d\n', size(TR.Points, 1));
fprintf('X 범위: [%.3f, %.3f]\n', min(TR.Points(:,1)), max(TR.Points(:,1)));
fprintf('Y 범위: [%.3f, %.3f]\n', min(TR.Points(:,2)), max(TR.Points(:,2)));
fprintf('Z 범위: [%.3f, %.3f]\n', min(TR.Points(:,3)), max(TR.Points(:,3)));

% 원하는 포인트 개수 설정 (삼각형 수에 비해 너무 적으면 문제 발생)
num_points = 50000; % 더 큰 값으로 변경

% 각 삼각형 면적 계산하여 면적 비례 샘플링
num_faces = size(TR.ConnectivityList, 1);
face_areas = zeros(num_faces, 1);

for i = 1:num_faces
    % 삼각형의 세 정점
    v1 = TR.Points(TR.ConnectivityList(i, 1), :);
    v2 = TR.Points(TR.ConnectivityList(i, 2), :);
    v3 = TR.Points(TR.ConnectivityList(i, 3), :);
    
    % 외적으로 면적 계산
    cross_vec = cross(v2 - v1, v3 - v1);
    face_areas(i) = 0.5 * norm(cross_vec);
end

% 각 면에서 샘플링할 포인트 수 결정
total_area = sum(face_areas);
points_per_face = round(face_areas / total_area * num_points);

% 디버깅: 포인트 할당 확인
fprintf('포인트가 할당된 면의 개수: %d\n', sum(points_per_face > 0));
fprintf('총 할당될 포인트 수: %d\n', sum(points_per_face));

% 포인트 샘플링
points = [];
total_points_to_sample = sum(points_per_face);
current_point_count = 0;

% 웨잇바 생성 - 포인트 샘플링용
wb2 = waitbar(0, '포인트 샘플링 중...', 'Name', 'Point Cloud 생성');

for i = 1:num_faces
    if points_per_face(i) > 0
        v1 = TR.Points(TR.ConnectivityList(i, 1), :);
        v2 = TR.Points(TR.ConnectivityList(i, 2), :);
        v3 = TR.Points(TR.ConnectivityList(i, 3), :);
        
        for j = 1:points_per_face(i)
            % 삼각형 내부 균등 샘플링
            r1 = rand();
            r2 = rand();
            if r1 + r2 > 1
                r1 = 1 - r1;
                r2 = 1 - r2;
            end
            
            point = v1 + r1 * (v2 - v1) + r2 * (v3 - v1);
            points = [points; point];
            current_point_count = current_point_count + 1;
        end
        
        % 웨잇바 업데이트 (100개 면마다)
        if mod(i, 100) == 0
            progress = current_point_count / total_points_to_sample;
            waitbar(progress, wb2, sprintf('포인트 샘플링 중... (%d/%d)', current_point_count, total_points_to_sample));
        end
    end
end

close(wb2);

% 디버깅: points 배열 확인
fprintf('points 배열 크기: [%d, %d]\n', size(points, 1), size(points, 2));

% points가 비어있지 않은지 확인
if isempty(points)
    error('포인트가 생성되지 않았습니다. num_points 값을 더 크게 설정하세요.');
end

% points가 올바른 크기인지 확인
if size(points, 2) ~= 3
    error('points 배열의 열 개수가 3이 아닙니다.');
end

% Point Cloud 생성
ptCloud = pointCloud(points);

% 결과 확인 및 시각화
fprintf('생성된 포인트 개수: %d\n', size(points, 1));
figure(2);
pcshow(ptCloud);
title('STL에서 생성된 Point Cloud');
xlabel('X [m]'); ylabel('Y [m]'); zlabel('Z [m]');

% Point Cloud 3가지 형식으로 저장
fprintf('\n=== Point Cloud 저장 ===\n');

% 1. PLY 형식으로 저장 (가장 범용적)
ply_filename = 'chief_pointcloud.ply';
try
    pcwrite(ptCloud, ply_filename);
    fprintf('✓ PLY 파일로 저장 완료: %s\n', ply_filename);
catch ME
    fprintf('✗ PLY 저장 실패: %s\n', ME.message);
end

% 2. PCD 형식으로 저장 (PCL 라이브러리 호환)
pcd_filename = 'chief_pointcloud.pcd';
try
    pcwrite(ptCloud, pcd_filename);
    fprintf('✓ PCD 파일로 저장 완료: %s\n', pcd_filename);
catch ME
    fprintf('✗ PCD 저장 실패: %s\n', ME.message);
end

% 3. MAT 파일로 저장 (MATLAB 전용, 빠른 로딩)
mat_filename = 'chief_pointcloud.mat';
try
    save(mat_filename, 'ptCloud', 'points');
    fprintf('✓ MAT 파일로 저장 완료: %s\n', mat_filename);
catch ME
    fprintf('✗ MAT 저장 실패: %s\n', ME.message);
end

fprintf('\n저장된 파일들:\n');
fprintf('- %s (범용적, CloudCompare/MeshLab 호환)\n', ply_filename);
fprintf('- %s (SLAM/로봇틱스 도구 호환)\n', pcd_filename);
fprintf('- %s (MATLAB 전용, 빠른 로딩)\n', mat_filename);
fprintf('모든 파일이 현재 작업 디렉토리에 저장되었습니다.\n');