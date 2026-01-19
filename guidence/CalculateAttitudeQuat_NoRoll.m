
function q_L2B = CalculateAttitudeQuat_NoRoll(pos, target, orbit_normal)
    % 쿼터니언 기반 자세 계산 (No-roll constraint)
    % pos: Deputy 위치 (LVLH)
    % target: 목표 위치 (Chief, 원점)
    % orbit_normal: 궤도 평면 법선 벡터
    
    % 위성 동체(Body) 좌표계 정의
    body_y = (target - pos) / norm(target - pos);  % Pointing direction
    body_x = cross(body_y, orbit_normal) / norm(cross(body_y, orbit_normal));  % No-roll constraint
    body_z = cross(body_x, body_y);
    
    R_L2B = [ body_x, body_y, body_z ]';
    
    % DCM을 쿼터니언으로 변환
    q_L2B = DCM2Quat(R_L2B);
    q_L2B = q_L2B / norm(q_L2B);  % 정규화
end