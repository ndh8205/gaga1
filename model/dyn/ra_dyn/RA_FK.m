% function FK = RA_FK(params, p_base, q_base, theta)
% % RA_FK: 위성 + 로봇팔 시스템의 Forward Kinematics
% %
% % 논문 참조: Zong et al., "Reactionless Control of Free-floating Space Manipulators"
% %           IEEE TAES, 2019 - Section II (System Modeling)
% %
% % =========================================================================
% % 입력 (Inputs):
% %   params - 시스템 파라미터 구조체 (params_init.m에서 생성)
% %   p_base - 위성 CoM 위치 (3x1) [m], 관성좌표계 표현
% %   q_base - 위성 자세 쿼터니언 (4x1) [qw; qx; qy; qz], Hamilton convention
% %   theta  - 관절 각도 벡터 (n_joints x 1) [rad]
% %
% % 출력 (Outputs):
% %   FK.R{i}       - 링크 i의 회전행렬 (3x3), 관성좌표계 기준
% %   FK.p_joint{i} - Joint i의 위치 (3x1) [m], 관성좌표계 표현
% %   FK.p_com{i}   - 링크 i의 CoM 위치 (3x1) [m], 관성좌표계 표현
% %   FK.k{i}       - Joint i의 회전축 단위벡터 (3x1), 관성좌표계 표현
% %   FK.p_ee       - End-effector 위치 (3x1) [m]
% %   FK.R_ee       - End-effector 자세 (3x3)
% %   FK.R_base     - 위성(Base) 회전행렬 (3x3)
% %   FK.p_mount    - 로봇팔 장착점 위치 (3x1) [m]
% %
% % =========================================================================
% % 좌표계 정의:
% %   - 관성좌표계 (Inertial frame, I): 고정 기준 좌표계
% %   - 위성 body frame (B_0): 위성 CoM에 부착
% %   - 링크 body frame (B_i): 각 링크에 부착
% %
% % =========================================================================
% 
% %% ========== 입력 검증 및 초기화 ==========
% 
% % 열벡터 보장
% p_base = p_base(:);
% q_base = q_base(:);
% theta = theta(:);
% 
% % 파라미터 추출
% n_joints = params.arm.n_joints;  % revolute joint 개수
% n_bodies = params.arm.n_bodies;  % 전체 body 개수
% r_mount = params.sat.r_mount(:); % 로봇팔 장착점 (위성 body frame 기준)
% 
% % 출력 구조체 초기화
% FK.R = cell(n_joints, 1);       % 각 링크의 회전행렬 (revolute만)
% FK.p_joint = cell(n_joints, 1); % 각 joint 위치 (revolute만)
% FK.p_com = cell(n_joints, 1);   % 각 링크 CoM 위치 (revolute만)
% FK.k = cell(n_joints, 1);       % 각 joint 회전축 (관성좌표계)
% 
% % 전체 body용 (fixed 포함)
% FK.R_all = cell(n_bodies, 1);      % 모든 body 회전행렬
% FK.p_com_all = cell(n_bodies, 1);  % 모든 body CoM 위치
% 
% %% ========== Step 1: 위성(Base) 자세 및 장착점 계산 ==========
% 
% % 위성의 회전행렬: Body frame -> Inertial frame
% % R_base * v^{body} = v^{inertial}
% R_base = GetDCM_QUAT(q_base);
% FK.R_base = R_base;
% 
% % 위성 CoM 위치 저장 (기준점)
% FK.p_base = p_base;
% 
% % 로봇팔 장착점 위치 (관성좌표계)
% % p_mount^I = p_base^I + R_base * r_mount^B
% p_mount = p_base + R_base * r_mount;
% FK.p_mount = p_mount;
% 
% %% ========== Step 2: 각 링크의 FK 계산 (재귀적) ==========
% 
% % 현재 프레임 상태 초기화 (장착점에서 시작)
% R_current = R_base;      % 현재 프레임의 회전행렬
% p_current = p_mount;     % 현재 프레임의 원점 위치
% 
% % joint 인덱스 (revolute joint만 카운트)
% joint_idx = 0;
% 
% % 모든 body 순회 (URDF 구조 기반)
% for i = 1:params.arm.n_bodies
% 
%     % 현재 링크 정보
%     link = params.arm.links(i);
% 
%     % Joint-to-Parent 변환 (4x4 동차변환)
%     % T_fixed: 부모 링크 끝 -> 현재 joint 위치
%     T_fixed = link.T_fixed;
%     R_fixed = T_fixed(1:3, 1:3);  % 회전 성분
%     t_fixed = T_fixed(1:3, 4);    % 변위 성분
% 
%     % Joint 위치 계산 (관성좌표계)
%     % p_joint^I = p_current^I + R_current * t_fixed^{local}
%     p_joint_i = p_current + R_current * t_fixed;
% 
%     % Joint 타입에 따른 처리
%     if strcmp(link.joint_type, 'revolute')
%         % Revolute joint인 경우
%         joint_idx = joint_idx + 1;
% 
%         % Joint 회전축 (부모 프레임 기준 -> 관성좌표계로 변환)
%         % k^I = R_current * R_fixed * axis^{local}
%         axis_local = link.joint_axis(:);
%         R_to_joint = R_current * R_fixed;
%         k_i = R_to_joint * axis_local;
%         k_i = k_i / norm(k_i);  % 단위벡터 보장
% 
%         % Joint 회전행렬 (axis-angle -> rotation matrix)
%         % Rodrigues' formula 사용
%         theta_i = theta(joint_idx);
%         R_joint = AnA2R(axis_local, theta_i);
% 
%         % 링크의 최종 회전행렬
%         % R_link^I = R_current * R_fixed * R_joint
%         R_link = R_to_joint * R_joint;
% 
%         % 링크 CoM 위치 (관성좌표계)
%         % CoM은 링크 로컬 좌표계 기준으로 정의됨
%         com_local = link.com(:);
%         p_com_i = p_joint_i + R_link * com_local;
% 
%         % 결과 저장 (revolute 전용)
%         FK.R{joint_idx} = R_link;
%         FK.p_joint{joint_idx} = p_joint_i;
%         FK.p_com{joint_idx} = p_com_i;
%         FK.k{joint_idx} = k_i;
% 
%         % 전체 body용 저장
%         FK.R_all{i} = R_link;
%         FK.p_com_all{i} = p_com_i;
% 
%         % 다음 링크를 위한 상태 업데이트
%         R_current = R_link;
%         p_current = p_joint_i;
% 
%     elseif strcmp(link.joint_type, 'fixed')
%         % Fixed joint인 경우 (base 또는 eef)
%         % 회전 없이 위치만 업데이트
%         R_link = R_current * R_fixed;
%         p_link = p_joint_i;
% 
%         % Fixed body CoM 위치
%         com_local = link.com(:);
%         p_com_i = p_link + R_link * com_local;
% 
%         % 전체 body용 저장
%         FK.R_all{i} = R_link;
%         FK.p_com_all{i} = p_com_i;
% 
%         % eef_link인 경우 End-effector로 저장
%         if contains(lower(link.name), 'eef')
%             FK.p_ee = p_link;
%             FK.R_ee = R_link;
%         end
% 
%         % 다음 링크를 위한 상태 업데이트
%         R_current = R_link;
%         p_current = p_link;
%     end
% end
% 
% %% ========== Step 3: End-effector Fallback ==========
% 
% % eef_link가 없는 경우 (fallback)
% if ~isfield(FK, 'p_ee') || isempty(FK.p_ee)
%     FK.p_ee = p_current;
%     FK.R_ee = R_current;
%     warning('RA_FK: eef_link not found. Using last frame as end-effector.');
% end
% 
% end
% 
% 
% %% ========== 보조 함수: Axis-Angle to Rotation Matrix ==========
% function R = AnA2R(axis, angle)
% % AnA2R: 회전축과 각도로부터 회전행렬 계산 (Rodrigues' formula)
% %
% % 입력:
% %   axis  - 회전축 단위벡터 (3x1)
% %   angle - 회전각 [rad]
% %
% % 출력:
% %   R - 회전행렬 (3x3)
% %
% % Rodrigues' formula:
% %   R = I + sin(θ) * [k]_× + (1 - cos(θ)) * [k]_×^2
% 
%     axis = axis(:) / norm(axis);  % 단위벡터 보장
% 
%     % 작은 각도 처리 (수치 안정성)
%     if abs(angle) < 1e-10
%         R = eye(3);
%         return;
%     end
% 
%     % Skew-symmetric matrix
%     K = skew3(axis);
% 
%     % Rodrigues' formula
%     R = eye(3) + sin(angle) * K + (1 - cos(angle)) * (K * K);
% end

function FK = RA_FK(params, p_base, q_base, theta)
% RA_FK: 위성 + 로봇팔 시스템의 Forward Kinematics
%
% 논문 참조: Zong et al., "Reactionless Control of Free-floating Space Manipulators"
%           IEEE TAES, 2019 - Section II (System Modeling)
%
% =========================================================================
% 입력 (Inputs):
%   params - 시스템 파라미터 구조체 (params_init.m에서 생성)
%   p_base - 위성 CoM 위치 (3x1) [m], 관성좌표계 표현
%   q_base - 위성 자세 쿼터니언 (4x1) [qw; qx; qy; qz], Hamilton convention
%   theta  - 관절 각도 벡터 (n_joints x 1) [rad]
%
% 출력 (Outputs):
%   FK.R{i}       - 링크 i의 회전행렬 (3x3), 관성좌표계 기준
%   FK.p_joint{i} - Joint i의 위치 (3x1) [m], 관성좌표계 표현
%   FK.p_com{i}   - 링크 i의 CoM 위치 (3x1) [m], 관성좌표계 표현
%   FK.k{i}       - Joint i의 회전축 단위벡터 (3x1), 관성좌표계 표현
%   FK.p_ee       - End-effector 위치 (3x1) [m]
%   FK.R_ee       - End-effector 자세 (3x3)
%   FK.R_base     - 위성(Base) 회전행렬 (3x3)
%   FK.p_mount    - 로봇팔 장착점 위치 (3x1) [m]
%
% =========================================================================
% 좌표계 정의:
%   - 관성좌표계 (Inertial frame, I): 고정 기준 좌표계
%   - 위성 body frame (B_0): 위성 CoM에 부착
%   - 링크 body frame (B_i): 각 링크에 부착
%
% =========================================================================

%% ========== 입력 검증 및 초기화 ==========

% 열벡터 보장
p_base = p_base(:);
q_base = q_base(:);
theta = theta(:);

% 파라미터 추출
n_joints = params.arm.n_joints;  % revolute joint 개수
n_bodies = params.arm.n_bodies;  % 전체 body 개수
r_mount = params.sat.r_mount(:); % 로봇팔 장착점 (위성 body frame 기준)

% 장착 회전 (하위 호환: 없으면 eye(3))
if isfield(params.sat, 'R_mount')
    R_mount = params.sat.R_mount;
else
    R_mount = eye(3);
end

% 출력 구조체 초기화
FK.R = cell(n_joints, 1);       % 각 링크의 회전행렬 (revolute만)
FK.p_joint = cell(n_joints, 1); % 각 joint 위치 (revolute만)
FK.p_com = cell(n_joints, 1);   % 각 링크 CoM 위치 (revolute만)
FK.k = cell(n_joints, 1);       % 각 joint 회전축 (관성좌표계)

% 전체 body용 (fixed 포함)
FK.R_all = cell(n_bodies, 1);      % 모든 body 회전행렬
FK.p_com_all = cell(n_bodies, 1);  % 모든 body CoM 위치

%% ========== Step 1: 위성(Base) 자세 및 장착점 계산 ==========

% 위성의 회전행렬: Body frame -> Inertial frame
% R_base * v^{body} = v^{inertial}
R_base = GetDCM_QUAT(q_base);
FK.R_base = R_base;

% 위성 CoM 위치 저장 (기준점)
FK.p_base = p_base;

% 로봇팔 장착점 위치 (관성좌표계)
% p_mount^I = p_base^I + R_base * r_mount^B
p_mount = p_base + R_base * r_mount;
FK.p_mount = p_mount;

% 장착 프레임 회전 (관성좌표계 기준)
% R_mount_I = R_base * R_mount
R_mount_I = R_base * R_mount;
FK.R_mount = R_mount_I;

%% ========== Step 2: 각 링크의 FK 계산 (재귀적) ==========

% 현재 프레임 상태 초기화 (장착점에서 시작)
% 장착 회전 적용: 로봇팔 로컬 좌표계 → 관성좌표계
R_current = R_mount_I;   % 장착 회전 반영
p_current = p_mount;     % 현재 프레임의 원점 위치

% joint 인덱스 (revolute joint만 카운트)
joint_idx = 0;

% 모든 body 순회 (URDF 구조 기반)
for i = 1:params.arm.n_bodies

    % 현재 링크 정보
    link = params.arm.links(i);

    % Joint-to-Parent 변환 (4x4 동차변환)
    % T_fixed: 부모 링크 끝 -> 현재 joint 위치
    T_fixed = link.T_fixed;
    R_fixed = T_fixed(1:3, 1:3);  % 회전 성분
    t_fixed = T_fixed(1:3, 4);    % 변위 성분

    % Joint 위치 계산 (관성좌표계)
    % p_joint^I = p_current^I + R_current * t_fixed^{local}
    p_joint_i = p_current + R_current * t_fixed;

    % Joint 타입에 따른 처리
    if strcmp(link.joint_type, 'revolute')
        % Revolute joint인 경우
        joint_idx = joint_idx + 1;

        % Joint 회전축 (부모 프레임 기준 -> 관성좌표계로 변환)
        % k^I = R_current * R_fixed * axis^{local}
        axis_local = link.joint_axis(:);
        R_to_joint = R_current * R_fixed;
        k_i = R_to_joint * axis_local;
        k_i = k_i / norm(k_i);  % 단위벡터 보장

        % Joint 회전행렬 (axis-angle -> rotation matrix)
        % Rodrigues' formula 사용
        theta_i = theta(joint_idx);
        R_joint = AnA2R(axis_local, theta_i);

        % 링크의 최종 회전행렬
        % R_link^I = R_current * R_fixed * R_joint
        R_link = R_to_joint * R_joint;

        % 링크 CoM 위치 (관성좌표계)
        % CoM은 링크 로컬 좌표계 기준으로 정의됨
        com_local = link.com(:);
        p_com_i = p_joint_i + R_link * com_local;

        % 결과 저장 (revolute 전용)
        FK.R{joint_idx} = R_link;
        FK.p_joint{joint_idx} = p_joint_i;
        FK.p_com{joint_idx} = p_com_i;
        FK.k{joint_idx} = k_i;
        
        % 전체 body용 저장
        FK.R_all{i} = R_link;
        FK.p_com_all{i} = p_com_i;

        % 다음 링크를 위한 상태 업데이트
        R_current = R_link;
        p_current = p_joint_i;

    elseif strcmp(link.joint_type, 'fixed')
        % Fixed joint인 경우 (base 또는 eef)
        % 회전 없이 위치만 업데이트
        R_link = R_current * R_fixed;
        p_link = p_joint_i;
        
        % Fixed body CoM 위치
        com_local = link.com(:);
        p_com_i = p_link + R_link * com_local;
        
        % 전체 body용 저장
        FK.R_all{i} = R_link;
        FK.p_com_all{i} = p_com_i;

        % eef_link인 경우 End-effector로 저장
        if contains(lower(link.name), 'eef')
            FK.p_ee = p_link;
            FK.R_ee = R_link;
        end

        % 다음 링크를 위한 상태 업데이트
        R_current = R_link;
        p_current = p_link;
    end
end

%% ========== Step 3: End-effector Fallback ==========

% eef_link가 없는 경우 (fallback)
if ~isfield(FK, 'p_ee') || isempty(FK.p_ee)
    FK.p_ee = p_current;
    FK.R_ee = R_current;
    warning('RA_FK: eef_link not found. Using last frame as end-effector.');
end

end


%% ========== 보조 함수: Axis-Angle to Rotation Matrix ==========
function R = AnA2R(axis, angle)
% AnA2R: 회전축과 각도로부터 회전행렬 계산 (Rodrigues' formula)
%
% 입력:
%   axis  - 회전축 단위벡터 (3x1)
%   angle - 회전각 [rad]
%
% 출력:
%   R - 회전행렬 (3x3)
%
% Rodrigues' formula:
%   R = I + sin(θ) * [k]_× + (1 - cos(θ)) * [k]_×^2

    axis = axis(:) / norm(axis);  % 단위벡터 보장

    % 작은 각도 처리 (수치 안정성)
    if abs(angle) < 1e-10
        R = eye(3);
        return;
    end

    % Skew-symmetric matrix
    K = skew3(axis);

    % Rodrigues' formula
    R = eye(3) + sin(angle) * K + (1 - cos(angle)) * (K * K);
end