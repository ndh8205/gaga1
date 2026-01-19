function c = RA_C(FK, params, v_base, omega_base, theta_dot)
% RA_C: Coriolis/Centrifugal 비선형 항 계산 (Recursive Newton-Euler Algorithm)
%
% ssh 코드 (RNEA_freefloating) 스타일 구현
% 위성을 포함한 전체 시스템 RNEA
%
% =========================================================================
% 동역학 방정식:
%   H * zeta_ddot + c = tau_gen
%
% =========================================================================
% 입력 (Inputs):
%   FK        - RA_FK 함수의 출력 구조체
%   params    - 시스템 파라미터 구조체
%   v_base    - Base CoM 선속도 (3x1) [m/s], 관성좌표계
%   omega_base- Base 각속도 (3x1) [rad/s], 관성좌표계
%   theta_dot - 관절 각속도 (n x 1) [rad/s]
%
% 출력 (Outputs):
%   c - 비선형 항 벡터 ((6+n) x 1)
%
% =========================================================================

%% ========== 입력 검증 및 초기화 ==========

v_base = v_base(:);
omega_base = omega_base(:);
theta_dot = theta_dot(:);

n_joints = params.arm.n_joints;  % 7
n_bodies = params.arm.n_bodies;  % 9 (base + link1~7 + eef)

% 출력 초기화
c = zeros(6 + n_joints, 1);

%% ========== 위치/회전 정보 준비 ==========
% ssh 스타일: Joint_pos, CoM_0_to_i 배열 생성
% 인덱스: 1=위성, 2=base, 3~9=link1~7

% Joint 위치 (n_joints+2 개: 장착점 + joint1~7 + eef)
Joint_pos = zeros(3, n_joints + 2);
Joint_pos(:, 1) = FK.p_mount;  % 장착점
for i = 1:n_joints
    Joint_pos(:, i+1) = FK.p_joint{i};
end
Joint_pos(:, n_joints+2) = FK.p_ee;  % EEF

% CoM 위치 (n_bodies+1 개: 위성 + base + link1~7 + eef)
% 인덱스 1 = 위성 CoM
CoM_pos = zeros(3, n_bodies + 1);
CoM_pos(:, 1) = FK.p_base;  % 위성 CoM
for i = 1:n_bodies
    CoM_pos(:, i+1) = FK.p_com_all{i};
end

% 회전행렬 (n_bodies+1 개)
R_all = cell(n_bodies + 1, 1);
R_all{1} = FK.R_base;  % 위성
for i = 1:n_bodies
    R_all{i+1} = FK.R_all{i};
end

% 관성텐서 (관성좌표계)
I_all = cell(n_bodies + 1, 1);
I_all{1} = FK.R_base * params.sat.I * FK.R_base';  % 위성
for i = 1:n_bodies
    I_all{i+1} = R_all{i+1} * params.arm.links(i).I * R_all{i+1}';
end

% 질량
m_all = zeros(n_bodies + 1, 1);
m_all(1) = params.sat.m;  % 위성
for i = 1:n_bodies
    m_all(i+1) = params.arm.links(i).m;
end

% 회전축 (n_joints 개)
k_all = zeros(3, n_joints);
for i = 1:n_joints
    k_all(:, i) = FK.k{i};
end

%% ========== Forward Pass: 속도/가속도 전파 ==========
% ssh 스타일: w, wd, vd, vc 배열 사용
% 인덱스: 1=관성좌표계, 2=위성, 3=base, 4~10=link1~7

% 각속도, 각가속도 (n_bodies+2 열)
w = zeros(3, n_bodies + 2);   % w(:,1)=0 (관성좌표계)
wd = zeros(3, n_bodies + 2);  % wd(:,1)=0

% 선가속도 (joint 기준), CoM 가속도
vd = zeros(3, n_bodies + 2);  % vd(:,1)=0 (관성좌표계 원점)
vc = zeros(3, n_bodies + 1);  % body CoM 가속도

% 힘/토크
F = zeros(3, n_bodies + 1);
N = zeros(3, n_bodies + 1);

% Backward pass용 벡터
p_ci = zeros(3, n_bodies + 1);  % CoM - Joint_next
p_ii = zeros(3, n_bodies + 1);  % Joint_next - Joint_prev

%% ----- Body 1: 위성 -----
% w(:,2) = 위성 각속도
w(:, 2) = omega_base;
wd(:, 2) = zeros(3, 1);  % 각가속도 입력 없음

% p_i = 위성 CoM
% p_ip = 장착점 (Joint_pos(:,1))
% p_ic = 위성 CoM (CoM_pos(:,1))
p_i = FK.p_base;
p_ip = Joint_pos(:, 1);
p_ic = CoM_pos(:, 1);

% 장착점의 선가속도 (원심가속도만)
vd(:, 2) = cross(wd(:, 2), p_ip - p_i) + cross(w(:, 2), cross(w(:, 2), p_ip - p_i));

% 위성 CoM 가속도
vc(:, 1) = vd(:, 2) + cross(wd(:, 2), p_ic - p_ip) + cross(w(:, 2), cross(w(:, 2), p_ic - p_ip));

% 위성 힘/토크
F(:, 1) = m_all(1) * vc(:, 1);
N(:, 1) = I_all{1} * wd(:, 2) + cross(w(:, 2), I_all{1} * w(:, 2));

% Backward용 벡터
p_ci(:, 1) = p_ic - p_ip;
p_ii(:, 1) = p_ip - p_i;

%% ----- Body 2: base link (fixed) -----
% 각속도 그대로 전달
w(:, 3) = w(:, 2);
wd(:, 3) = wd(:, 2);

p_i = Joint_pos(:, 1);   % 이전: 장착점
p_ip = Joint_pos(:, 2);  % 다음: joint1
p_ic = CoM_pos(:, 2);    % base CoM

vd(:, 3) = vd(:, 2) + cross(wd(:, 3), p_ip - p_i) + cross(w(:, 3), cross(w(:, 3), p_ip - p_i));
vc(:, 2) = vd(:, 3) + cross(wd(:, 3), p_ic - p_ip) + cross(w(:, 3), cross(w(:, 3), p_ic - p_ip));

F(:, 2) = m_all(2) * vc(:, 2);
N(:, 2) = I_all{2} * wd(:, 3) + cross(w(:, 3), I_all{2} * w(:, 3));

p_ci(:, 2) = p_ic - p_ip;
p_ii(:, 2) = p_ip - p_i;

%% ----- Body 3~9: link1~7 (revolute) -----
for i = 3:n_bodies  % i = 3~9 (link1~7), eef는 질량 0이므로 스킵 가능
    joint_idx = i - 2;  % 1~7
    
    p_i = Joint_pos(:, i-1);   % 이전 joint
    p_ip = Joint_pos(:, i);    % 현재 joint
    p_ic = CoM_pos(:, i);      % 현재 body CoM
    
    % 각속도
    w(:, i+1) = w(:, i) + theta_dot(joint_idx) * k_all(:, joint_idx);
    wd(:, i+1) = wd(:, i) + cross(w(:, i), theta_dot(joint_idx) * k_all(:, joint_idx));
    
    % 선가속도
    vd(:, i+1) = vd(:, i) + cross(wd(:, i+1), p_ip - p_i) + cross(w(:, i+1), cross(w(:, i+1), p_ip - p_i));
    vc(:, i) = vd(:, i+1) + cross(wd(:, i+1), p_ic - p_ip) + cross(w(:, i+1), cross(w(:, i+1), p_ic - p_ip));
    
    % 힘/토크
    F(:, i) = m_all(i) * vc(:, i);
    N(:, i) = I_all{i} * wd(:, i+1) + cross(w(:, i+1), I_all{i} * w(:, i+1));
    
    % Backward용 벡터
    p_ci(:, i) = p_ic - p_ip;
    p_ii(:, i) = p_ip - p_i;
end

%% ========== Backward Pass: 힘/토크 전파 ==========
tau = zeros(n_joints, 1);

f_next = zeros(3, 1);
n_next = zeros(3, 1);

% link7 → link1 (i = 9 → 3)
for i = n_bodies:-1:3
    joint_idx = i - 2;  % 7 → 1
    
    % 질량 0인 body 스킵
    if m_all(i) < 1e-10
        continue;
    end
    
    f = f_next + F(:, i);
    n = N(:, i) + n_next + cross(f_next, p_ci(:, i)) + cross(p_ci(:, i) + p_ii(:, i), f);
    
    % Joint 토크 추출
    tau(joint_idx) = n' * k_all(:, joint_idx);
    
    f_next = f;
    n_next = n;
end

% base link (i = 2)
f = f_next + F(:, 2);
n = N(:, 2) + n_next + cross(f_next, p_ci(:, 2)) + cross(p_ci(:, 2) + p_ii(:, 2), f);
f_next = f;
n_next = n;

% 위성 (i = 1)
f = f_next + F(:, 1);
n = N(:, 1) + n_next + cross(f_next, p_ci(:, 1)) + cross(p_ci(:, 1) + p_ii(:, 1), f);
f_next = f;
n_next = n;

%% ========== 출력 조립 ==========
% c = [f_base; n_base; tau]
c(1:3) = f_next;
c(4:6) = n_next;
c(7:end) = tau;

end