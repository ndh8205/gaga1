function H = RA_MM_dual(FK_dual, params)
% RA_MM_dual: 양팔 시스템 일반화 관성행렬 (Mass Matrix) 계산
%
% 논문 참조: Zong et al., "Reactionless Control of Free-floating Space Manipulators"
%           IEEE TAES, 2019 - Eq. 14-24 확장
%
% =========================================================================
% 일반화 관성 행렬 구조 (20x20):
%
%   H = [ H_b      H_bm_L    H_bm_R  ]   6
%       [ H_bm_L'  H_m_L     0       ]   7
%       [ H_bm_R'  0         H_m_R   ]   7
%           6        7         7
%
% 핵심: 
%   - 위성 관성은 1번만
%   - 시스템 CoM은 양팔 전체 (fixed body 포함)
%   - M, H_w, H_vw: 모든 body 포함 (fixed 포함)
%   - H_m, H_wq, J_Tw: revolute만
%
% =========================================================================
% 입력 (Inputs):
%   FK_dual - RA_FK_dual 함수의 출력 구조체
%   params  - 시스템 파라미터 구조체
%
% 출력 (Outputs):
%   H - 일반화 관성 행렬 (20x20)
%       일반화 속도: [v_b; w_b; theta_dot_L; theta_dot_R]
%
% =========================================================================

%% ========== 파라미터 추출 ==========

% 위성 파라미터
m_sat = params.sat.m;
I_sat = params.sat.I;

% 로봇팔 파라미터
n_joints = params.arm.n_joints;  % 7
n_bodies = params.arm.n_bodies;  % 9

% FK 결과 추출
R_base = FK_dual.R_base;
p_base = FK_dual.p_base;

%% ========== 위성 관성 (관성좌표계) ==========
I_sat_I = R_base * I_sat * R_base';

%% ========== 왼팔 모든 body 정보 (fixed 포함) ==========
[m_all_L, I_all_L, r_0i_all_L] = compute_all_body_properties(FK_dual.L, params, p_base);

%% ========== 오른팔 모든 body 정보 (fixed 포함) ==========
[m_all_R, I_all_R, r_0i_all_R] = compute_all_body_properties(FK_dual.R, params, p_base);

%% ========== 왼팔 revolute 정보 (Jacobian용) ==========
[m_rev_L, I_rev_L, r_0i_rev_L, J_T_L, J_R_L] = compute_revolute_properties(FK_dual.L, params, p_base);

%% ========== 오른팔 revolute 정보 (Jacobian용) ==========
[m_rev_R, I_rev_R, r_0i_rev_R, J_T_R, J_R_R] = compute_revolute_properties(FK_dual.R, params, p_base);

%% ========== 전체 질량 (모든 body 포함) ==========
M = m_sat + sum(m_all_L) + sum(m_all_R);

%% ========== 시스템 CoM (양팔 전체, 모든 body 포함) ==========
r_0g_sum = zeros(3, 1);
for i = 1:n_bodies
    r_0g_sum = r_0g_sum + m_all_L(i) * r_0i_all_L{i};
    r_0g_sum = r_0g_sum + m_all_R(i) * r_0i_all_R{i};
end
r_0g = r_0g_sum / M;

%% ========== H_vw (속도-각속도 결합, 3x3) ==========
H_vw = -M * skew3(r_0g);

%% ========== H_w (회전 관성, 3x3) - 모든 body 포함 ==========
H_w = I_sat_I;

for i = 1:n_bodies
    % 왼팔 기여 (모든 body)
    H_w = H_w + I_all_L{i} + m_all_L(i) * skew3(r_0i_all_L{i})' * skew3(r_0i_all_L{i});
    % 오른팔 기여 (모든 body)
    H_w = H_w + I_all_R{i} + m_all_R(i) * skew3(r_0i_all_R{i})' * skew3(r_0i_all_R{i});
end

%% ========== H_m_L (왼팔 관성, 7x7) - revolute만 ==========
H_m_L = zeros(n_joints, n_joints);
for i = 1:n_joints
    H_m_L = H_m_L + m_rev_L(i) * (J_T_L{i}' * J_T_L{i}) + J_R_L{i}' * I_rev_L{i} * J_R_L{i};
end

%% ========== H_m_R (오른팔 관성, 7x7) - revolute만 ==========
H_m_R = zeros(n_joints, n_joints);
for i = 1:n_joints
    H_m_R = H_m_R + m_rev_R(i) * (J_T_R{i}' * J_T_R{i}) + J_R_R{i}' * I_rev_R{i} * J_R_R{i};
end

%% ========== H_wq_L (각속도-왼팔관절 결합, 3x7) - revolute만 ==========
H_wq_L = zeros(3, n_joints);
for i = 1:n_joints
    H_wq_L = H_wq_L + m_rev_L(i) * skew3(r_0i_rev_L{i}) * J_T_L{i} + I_rev_L{i} * J_R_L{i};
end

%% ========== H_wq_R (각속도-오른팔관절 결합, 3x7) - revolute만 ==========
H_wq_R = zeros(3, n_joints);
for i = 1:n_joints
    H_wq_R = H_wq_R + m_rev_R(i) * skew3(r_0i_rev_R{i}) * J_T_R{i} + I_rev_R{i} * J_R_R{i};
end

%% ========== J_Tw_L (관절-선속도 결합, 3x7) - revolute만 ==========
J_Tw_L = zeros(3, n_joints);
for i = 1:n_joints
    J_Tw_L = J_Tw_L + m_rev_L(i) * J_T_L{i};
end

%% ========== J_Tw_R (관절-선속도 결합, 3x7) - revolute만 ==========
J_Tw_R = zeros(3, n_joints);
for i = 1:n_joints
    J_Tw_R = J_Tw_R + m_rev_R(i) * J_T_R{i};
end

%% ========== H_bm 조립 ==========
H_bm_L = [J_Tw_L; H_wq_L];  % 6x7
H_bm_R = [J_Tw_R; H_wq_R];  % 6x7

%% ========== H_b 조립 (6x6) ==========
H_b = [M * eye(3), H_vw;
       H_vw',      H_w];

%% ========== 전체 H 조립 (20x20) ==========
H = zeros(6 + 2*n_joints, 6 + 2*n_joints);

% H_b (6x6)
H(1:6, 1:6) = H_b;

% H_bm_L (6x7) 및 전치
H(1:6, 7:13) = H_bm_L;
H(7:13, 1:6) = H_bm_L';

% H_bm_R (6x7) 및 전치
H(1:6, 14:20) = H_bm_R;
H(14:20, 1:6) = H_bm_R';

% H_m_L (7x7)
H(7:13, 7:13) = H_m_L;

% H_m_R (7x7)
H(14:20, 14:20) = H_m_R;

end


%% ========== 보조 함수: 모든 body 속성 (fixed 포함) ==========
function [m_all, I_all, r_0i_all] = compute_all_body_properties(FK, params, p_base)
% 모든 body의 질량, 관성, 위치 반환 (M, H_w, H_vw 계산용)

n_bodies = params.arm.n_bodies;

m_all = zeros(n_bodies, 1);
I_all = cell(n_bodies, 1);
r_0i_all = cell(n_bodies, 1);

for i = 1:n_bodies
    link = params.arm.links(i);
    
    % 질량
    m_all(i) = link.m;
    
    % 관성텐서 (관성좌표계 변환)
    R_i = FK.R_all{i};
    I_all{i} = R_i * link.I * R_i';
    
    % 위성 CoM에서 링크 CoM까지 벡터
    p_com_i = FK.p_com_all{i};
    r_0i_all{i} = p_com_i - p_base;
end

end


%% ========== 보조 함수: revolute body 속성 (Jacobian 포함) ==========
function [m_rev, I_rev, r_0i_rev, J_T, J_R] = compute_revolute_properties(FK, params, p_base)
% revolute body의 속성 및 Jacobian 반환 (H_m, H_wq, J_Tw 계산용)

n_joints = params.arm.n_joints;
n_bodies = params.arm.n_bodies;

m_rev = zeros(n_joints, 1);
I_rev = cell(n_joints, 1);
r_0i_rev = cell(n_joints, 1);
J_T = cell(n_joints, 1);
J_R = cell(n_joints, 1);

joint_count = 0;

for i = 1:n_bodies
    link = params.arm.links(i);
    
    if strcmp(link.joint_type, 'revolute')
        joint_count = joint_count + 1;
        
        % 질량
        m_rev(joint_count) = link.m;
        
        % 관성텐서 (관성좌표계 변환)
        R_i = FK.R_all{i};
        I_rev{joint_count} = R_i * link.I * R_i';
        
        % 위성 CoM에서 링크 CoM까지 벡터
        p_com_i = FK.p_com_all{i};
        r_0i_rev{joint_count} = p_com_i - p_base;
        
        % 선속도 Jacobian (3 x n_joints)
        J_T{joint_count} = zeros(3, n_joints);
        for j = 1:joint_count
            k_j = FK.k{j};
            r_ji = p_com_i - FK.p_joint{j};
            J_T{joint_count}(:, j) = cross(k_j, r_ji);
        end
        
        % 각속도 Jacobian (3 x n_joints)
        J_R{joint_count} = zeros(3, n_joints);
        for j = 1:joint_count
            J_R{joint_count}(:, j) = FK.k{j};
        end
    end
end

end