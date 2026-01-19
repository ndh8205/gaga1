function H = RA_MM(FK, params)
% RA_MM: 일반화 관성 행렬 (Mass Matrix) 계산
%
% 논문 참조: Zong et al., "Reactionless Control of Free-floating Space Manipulators"
%           IEEE TAES, 2019 - Eq. 14-24
%
% =========================================================================
% 일반화 관성 행렬 구조 (Eq. 14):
%
%   H' = [ M*I_3        H_vw'         J_Tw'^T     ]
%        [ H_vw'^T      H_w'          H_wq'       ]
%        [ J_Tw'        H_wq'^T       H_m'        ]
%
%   크기: (6+n) x (6+n)
%   - M*I_3: 3x3 (전체 질량)
%   - H_vw': 3x3 (속도-각속도 결합)
%   - J_Tw': nx3 (관절-선속도 결합)
%   - H_w': 3x3 (회전 관성)
%   - H_wq': 3xn (각속도-관절 결합)
%   - H_m': nxn (매니퓰레이터 관성)
%
% =========================================================================
% 입력 (Inputs):
%   FK     - RA_FK 함수의 출력 구조체
%   params - 시스템 파라미터 구조체
%
% 출력 (Outputs):
%   H - 일반화 관성 행렬 ((6+n) x (6+n))
%       일반화 속도: [v_0; w_base; theta_dot]
%
% =========================================================================

%% ========== 파라미터 추출 ==========

% 위성 파라미터
m_sat = params.sat.m;           % 위성 질량
I_sat = params.sat.I;           % 위성 관성텐서 (body frame)

% 로봇팔 파라미터
n_joints = params.arm.n_joints; % 관절 수

% FK 결과 추출
R_base = FK.R_base;             % 위성 회전행렬
p_mount = FK.p_mount;           % 장착점 위치 (기준점)

%% ========== 링크별 Jacobian 계산 (J_Ti, J_Ri) ==========
%
% 각 링크 i의 CoM 속도:
%   v_ci = v_0 + w_base x r_0i + sum_{j=1}^{i} (k_j x r_ji) * theta_dot_j
%
% 행렬 형태:
%   v_ci = [I_3, -[r_0i]_x] * [v_0; w_base] + J_Ti * theta_dot
%
% 각 링크 i의 각속도:
%   w_i = w_base + sum_{j=1}^{i} k_j * theta_dot_j
%
% 행렬 형태:
%   w_i = [0_3, I_3] * [v_0; w_base] + J_Ri * theta_dot

% 링크별 Jacobian 저장 (revolute만)
J_T = cell(n_joints, 1);  % 선속도 Jacobian (3 x n)
J_R = cell(n_joints, 1);  % 각속도 Jacobian (3 x n)

% 전체 body 관성 정보 (fixed 포함)
n_bodies = params.arm.n_bodies;
I_all = cell(n_bodies, 1);      % 관성텐서 (관성좌표계)
m_all = zeros(n_bodies, 1);     % 질량
r_0i_all = cell(n_bodies, 1);   % 기준점에서 body CoM까지 벡터

% Revolute body 정보 (H_m, H_wq, J_Tw용)
I_rev = cell(n_joints, 1);      % revolute body 관성텐서
m_rev = zeros(n_joints, 1);     % revolute body 질량
r_0i_rev = cell(n_joints, 1);   % revolute body r_0i

% 모든 body 순회 (fixed 포함)
for i = 1:n_bodies
    link = params.arm.links(i);
    m_all(i) = link.m;
    R_i = FK.R_all{i};
    I_all{i} = R_i * link.I * R_i';
    r_0i_all{i} = FK.p_com_all{i} - FK.p_base;
end

% Revolute body만 Jacobian 및 별도 정보 저장
joint_count = 0;
for i = 1:n_bodies
    link = params.arm.links(i);
    
    if strcmp(link.joint_type, 'revolute')
        joint_count = joint_count + 1;
        
        % Revolute body 정보 저장
        m_rev(joint_count) = m_all(i);
        I_rev{joint_count} = I_all{i};
        r_0i_rev{joint_count} = r_0i_all{i};
        
        % 선속도 Jacobian (J_Ti): 3 x n
        J_T{joint_count} = zeros(3, n_joints);
        for j = 1:joint_count
            k_j = FK.k{j};
            r_ji = FK.p_com_all{i} - FK.p_joint{j};
            J_T{joint_count}(:, j) = cross(k_j, r_ji);
        end
        
        % 각속도 Jacobian (J_Ri): 3 x n
        J_R{joint_count} = zeros(3, n_joints);
        for j = 1:joint_count
            J_R{joint_count}(:, j) = FK.k{j};
        end
    end
end

%% ========== 전체 질량 (Eq. 15) ==========
% M = m_sat + sum(m_i) - 모든 body 포함

M = m_sat + sum(m_all);

%% ========== 시스템 CoM 위치 계산 ==========
% 속도-각속도 결합항 계산을 위해 필요
% 기준점 = 위성 CoM → 위성의 r = 0

% 시스템 전체 CoM 위치 (기준점 = 위성 CoM)
r_0g_sum = zeros(3,1);  % m_sat * 0 = 0
for i = 1:n_bodies
    r_0g_sum = r_0g_sum + m_all(i) * r_0i_all{i};
end
r_0g = r_0g_sum / M;

%% ========== H_vw' (속도-각속도 결합) - Eq. 18 ==========
% H_vw' = M * [r_0g]_x^T = -M * [r_0g]_x
%
% 의미: Base 각속도 w_base가 시스템 선운동량에 미치는 영향

H_vw = -M * skew3(r_0g);

%% ========== H_w' (회전 관성) - Eq. 16 ==========
% H_w' = I_sat^I + sum_i { I_i^I + m_i*[r_0i]_x^T*[r_0i]_x }
%
% 기준점 = 위성 CoM → 위성의 위치 기여 = 0 (관성텐서만)
% 모든 body 포함 (fixed 포함)

% 위성 기여 (관성텐서만, 위치 기여 없음)
I_sat_inertial = R_base * I_sat * R_base';  % 관성좌표계 변환
H_w = I_sat_inertial;

% 모든 body 기여 (fixed 포함)
for i = 1:n_bodies
    H_w = H_w + I_all{i} + m_all(i) * skew3(r_0i_all{i})' * skew3(r_0i_all{i});
end

%% ========== H_m' (매니퓰레이터 관성) - Eq. 17 ==========
% H_m' = sum_i { m_i * J_Ti^T * J_Ti + J_Ri^T * I_i^I * J_Ri }
%
% 의미: 관절 가속도가 관절 토크에 미치는 영향 (revolute만)

H_m = zeros(n_joints, n_joints);
for i = 1:n_joints
    H_m = H_m + m_rev(i) * (J_T{i}' * J_T{i}) + J_R{i}' * I_rev{i} * J_R{i};
end

%% ========== H_wq' (각속도-관절 결합) - Eq. 19 ==========
% H_wq' = sum_i { m_i * [r_0i]_x * J_Ti + I_i^I * J_Ri }
%
% 의미: 관절 가속도가 Base 각운동량에 미치는 영향 (revolute만)

H_wq = zeros(3, n_joints);
for i = 1:n_joints
    H_wq = H_wq + m_rev(i) * skew3(r_0i_rev{i}) * J_T{i} + I_rev{i} * J_R{i};
end

%% ========== J_Tw' (관절-선속도 결합) - Eq. 20 ==========
% J_Tw' = sum_i { m_i * J_Ti }
%
% 의미: 관절 가속도가 시스템 선운동량에 미치는 영향 (revolute만)

J_Tw = zeros(3, n_joints);
for i = 1:n_joints
    J_Tw = J_Tw + m_rev(i) * J_T{i};
end

%% ========== 전체 관성 행렬 조립 ==========
%
%   H' = [ M*I_3        H_vw'         J_Tw'^T     ]   3
%        [ H_vw'^T      H_w'          H_wq'       ]   3
%        [ J_Tw'        H_wq'^T       H_m'        ]   n
%            3            3             n

H = zeros(6 + n_joints, 6 + n_joints);

% 좌상단 (3x3): M * I_3
H(1:3, 1:3) = M * eye(3);

% 우상단 (3x3): H_vw'
H(1:3, 4:6) = H_vw;

% 좌상단-우측 (3xn): J_Tw'^T = J_Tw (논문 표기상 J_Tw'는 nx3, 여기서 J_Tw는 3xn)
H(1:3, 7:end) = J_Tw;

% 좌측-중간 (3x3): H_vw'^T
H(4:6, 1:3) = H_vw';

% 중앙 (3x3): H_w'
H(4:6, 4:6) = H_w;

% 중앙-우측 (3xn): H_wq'
H(4:6, 7:end) = H_wq;

% 좌하단 (nx3): J_Tw' (3xn의 transpose = nx3)
H(7:end, 1:3) = J_Tw';

% 중하단 (nx3): H_wq'^T
H(7:end, 4:6) = H_wq';

% 우하단 (nxn): H_m'
H(7:end, 7:end) = H_m;

end


%% ========== 보조 함수: Revolute Joint 인덱스 찾기 ==========
function idx = findRevoluteIndex(params, rev_num)
% findRevoluteIndex: rev_num번째 revolute joint에 해당하는 링크 인덱스 반환
%
% params.arm.links에는 fixed joint도 포함되어 있으므로,
% revolute joint만 카운트하여 해당 인덱스 반환

    count = 0;
    for i = 1:params.arm.n_bodies
        if strcmp(params.arm.links(i).joint_type, 'revolute')
            count = count + 1;
            if count == rev_num
                idx = i;
                return;
            end
        end
    end
    error('Revolute joint index %d not found', rev_num);
end