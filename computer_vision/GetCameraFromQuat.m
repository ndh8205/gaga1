function [R_L2C, R_B2L] = GetCameraFromQuat(q_L2B, pos, orbit_normal)
    % q_L2B : LVLH -> Body quaternion (234 규약)
    % 반환: R_cam_from_lvlh = R_L2C,  R_body_in_lvlh = R_B2L

    R_L2B = GetDCM_QUAT(q_L2B);
    R_B2L = R_L2B';

    % Body 축 (LVLH에서 본)
    bx = R_B2L(:,1);
    by = R_B2L(:,2);
    bz = R_B2L(:,3);

    % 카메라 축 정의: z_cam = +Y_B (광축), x_cam = +X_B (오른쪽)
    zc = by;   zc = zc / norm(zc + eps);
    xc = bx;   xc = xc / norm(xc + eps);

    % y_cam = z_cam x x_cam  (오른손좌표 유지)
    yc = cross(zc, xc);
    yc = yc / norm(yc + eps);

    % 재직교화(수치오차 억제): x를 y,z에 대해 재투영 제거
    xc = xc - yc*(yc.'*xc) - zc*(zc.'*xc); xc = xc / norm(xc + eps);
    % (필요 시 yc도 한 번 더 정규화)
    yc = cross(zc, xc); yc = yc / norm(yc + eps);

    % det 보정(드물게 -1 나올 때)
    if det([xc yc zc]) < 0
        yc = -yc;
    end

    % R_L2C: LVLH 좌표 -> Camera 좌표
    R_C2L = [ xc, yc, zc ];
    R_L2C = R_C2L';
end 
