function M = create_M_matrix(ra, de, roll)
% create_M_matrix - 회전 행렬 생성
%
% Inputs:
%   ra   - Right Ascension (radians)
%   de   - Declination (radians)
%   roll - Roll angle (radians)
%
% Output:
%   M - 3x3 rotation matrix

    ra_exp = ra - (pi/2);
    de_exp = de + (pi/2);

    M1 = [cos(ra_exp), -sin(ra_exp), 0;
          sin(ra_exp),  cos(ra_exp), 0;
          0,            0,           1];

    M2 = [1,  0,            0;
          0,  cos(de_exp), -sin(de_exp);
          0,  sin(de_exp),  cos(de_exp)];

    M3 = [cos(roll), -sin(roll), 0;
          sin(roll),  cos(roll), 0;
          0,          0,         1];

    M = M1 * M2 * M3;
end
