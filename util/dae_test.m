clc; clear; close all;
addpath(genpath('D:\pj2025\space_challenge'));

%% 1. 경로 및 파일 설정 (여기만 수정하세요)
fullFilePath = 'D:\pj2025\space_challenge\model\modeling_3d\intel_dummy.dae'; 

%% 2. 경로 처리 (자동)
% 파일의 폴더 경로와 파일명을 분리합니다.
[folderPath, fileName, fileExt] = fileparts(fullFilePath);

% DAE 파일과 같은 폴더에 있는 텍스처 이미지를 읽을 수 있도록 경로를 추가합니다.
if ~isempty(folderPath)
    addpath(folderPath);
end

% 확장자가 포함된 파일명 생성
targetFile = [fileName, fileExt];

%% 3. 시각화 로직 (Method 1)
try
    % 가상의 로봇 바디 생성
    body = rigidBody('visualBody');

    % DAE 파일을 시각적 요소(Mesh)로 추가
    % addpath를 했으므로 파일명만 넣으면 됩니다.
    addVisual(body, 'Mesh', targetFile);

    % 로봇 트리에 바디 부착 ('base'는 기준 좌표계)
    robot = rigidBodyTree;
    addBody(robot, body, 'base');

    % 시각화 출력
    figure;
    show(robot, 'Visuals', 'on', 'Collisions', 'off'); 

    % 뷰 설정 (보기 좋게 다듬기)
    axis equal;      % 비율 유지
    grid on;         % 그리드 켜기
    camlight('headlight'); % 조명 추가 (색상 잘 보이게)
    lighting gouraud;      % 부드러운 음영
    title(['View: ', targetFile], 'Interpreter', 'none');

    disp('시각화 성공! 마우스로 회전해보세요.');

catch ME
    warning('파일을 불러오는 중 오류가 발생했습니다.');
    disp(ME.message);
    disp('팁: Robotics System Toolbox가 설치되어 있는지 확인해주세요.');
end