function params = params_init_old(urdf_path)
% params_init: 위성 + 로봇팔 시스템 파라미터 초기화
%
% Input:
%   urdf_path - URDF 파일 경로
%
% Output:
%   params.sat  - 위성 파라미터
%   params.arm  - 로봇팔 파라미터 (URDF 자동 추출)
%   params.env  - 환경 파라미터

%% ========== SATELLITE ==========
params.sat.m = 500;  % [kg]
params.sat.I = [260, -0.2, 0.6;
               -0.2,  280,   4;
                0.6,    4, 170];  % [kg*m^2]
params.sat.size = [1; 1; 1];  % [m]
params.sat.r_mount = [0; 0; 0.5];  % 로봇팔 장착점 (위성 CoM 기준)

%% ========== ROBOT ARM (URDF 자동 추출) ==========
robot = importrobot(urdf_path, 'DataFormat', 'column');
robot.Gravity = [0; 0; 0];

params.arm.robot = robot;
params.arm.n_bodies = length(robot.Bodies);
params.arm.n_joints = 0;

% 각 링크 파라미터 추출
for i = 1:params.arm.n_bodies
    body = robot.Bodies{i};

    % 기본 정보
    params.arm.links(i).name = body.Name;
    params.arm.links(i).m = body.Mass;
    params.arm.links(i).com = body.CenterOfMass(:);

    % 관성텐서 (6x1 → 3x3)
    % MATLAB: [Ixx Iyy Izz Iyz Ixz Ixy]
    % 주의: MATLAB importrobot은 원점 기준 관성텐서를 저장함
    % Inverse PAT 적용하여 CoM 기준으로 변환
    I_vec = body.Inertia;
    I_origin = [I_vec(1), I_vec(6), I_vec(5);
                I_vec(6), I_vec(2), I_vec(4);
                I_vec(5), I_vec(4), I_vec(3)];

    % Inverse Parallel Axis Theorem: I_com = I_origin - m*(|r|²*I - r*r')
    m = body.Mass;
    r = params.arm.links(i).com;
    if m > 0
        r_sq = r' * r;
        I_com = I_origin - m * (r_sq * eye(3) - r * r');
    else
        I_com = I_origin;
    end
    params.arm.links(i).I = I_com;

    % 부모 링크
    if ~isempty(body.Parent)
        params.arm.links(i).parent = body.Parent.Name;
    else
        params.arm.links(i).parent = '';
    end

    % 관절 정보
    params.arm.links(i).joint_name = body.Joint.Name;
    params.arm.links(i).joint_type = body.Joint.Type;
    params.arm.links(i).joint_axis = body.Joint.JointAxis(:);
    params.arm.links(i).T_fixed = body.Joint.JointToParentTransform;

    % revolute 관절 카운트
    if strcmp(body.Joint.Type, 'revolute')
        params.arm.n_joints = params.arm.n_joints + 1;
    end
end

%% ========== ENVIRONMENT ==========
params.env.g = [0; 0; 0];  % 우주환경

end