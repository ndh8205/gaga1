function [r, v] = coe2rv(coe, mu)
%COE2RV Convert Classical Orbital Elements to Position and Velocity Vectors
%
%   [r, v] = COE2RV(coe, mu) converts classical orbital elements to 
%   position and velocity vectors in the geocentric equatorial frame.
%
%   INPUTS:
%       coe - Classical orbital elements vector [6x1]:
%             coe(1) = h     - Specific angular momentum (km²/s)
%             coe(2) = e     - Eccentricity (dimensionless)
%             coe(3) = RA    - Right ascension of ascending node (rad)
%             coe(4) = incl  - Inclination (rad)
%             coe(5) = w     - Argument of periapsis (rad)
%             coe(6) = TA    - True anomaly (rad)
%       mu  - Gravitational parameter (km³/s²)
%
%   OUTPUTS:
%       r   - Position vector in geocentric equatorial frame [3x1] (km)
%       v   - Velocity vector in geocentric equatorial frame [3x1] (km/s)
%
%   ALGORITHM:
%       1. Extract orbital elements from input vector
%       2. Calculate position and velocity in perifocal coordinate system
%       3. Create rotation matrices for coordinate transformation
%       4. Transform vectors to geocentric equatorial frame
%
%   REFERENCE: Curtis, H. D. "Orbital Mechanics for Engineering Students"

% ========================================================================
% STEP 1: Extract orbital elements from input vector
% ========================================================================
h    = coe(1);    % Specific angular momentum [km²/s]
e    = coe(2);    % Eccentricity [-]
RA   = coe(3);    % Right ascension of ascending node [rad]
incl = coe(4);    % Inclination [rad]
w    = coe(5);    % Argument of periapsis [rad]
TA   = coe(6);    % True anomaly [rad]

% ========================================================================
% STEP 2: Calculate position and velocity in perifocal coordinate system
% ========================================================================
% Position vector in perifocal frame (Equations 4.45)
% The perifocal frame has its x-axis pointing toward periapsis
rp = (h^2/mu) * (1/(1 + e*cos(TA))) * ...
     (cos(TA)*[1; 0; 0] + sin(TA)*[0; 1; 0]);

% Velocity vector in perifocal frame (Equations 4.46)
% Velocity is perpendicular to position in the orbital plane
vp = (mu/h) * (-sin(TA)*[1; 0; 0] + (e + cos(TA))*[0; 1; 0]);

% ========================================================================
% STEP 3: Create rotation matrices for coordinate transformation
% ========================================================================
% Rotation matrix about Z-axis by right ascension angle (Equation 4.34)
% Rotates from inertial frame to node line
R3_W = [cos(RA)   sin(RA)  0;
        -sin(RA)  cos(RA)  0;
        0         0        1];

% Rotation matrix about X-axis by inclination angle (Equation 4.32)
% Rotates about node line by inclination angle
R1_i = [1  0           0;
        0  cos(incl)   sin(incl);
        0  -sin(incl)  cos(incl)];

% Rotation matrix about Z-axis by argument of periapsis (Equation 4.34)
% Rotates from ascending node to periapsis direction
R3_w = [cos(w)   sin(w)  0;
        -sin(w)  cos(w)  0;
        0        0       1];

% ========================================================================
% STEP 4: Transform vectors to geocentric equatorial frame
% ========================================================================
% Combined transformation matrix from perifocal to geocentric equatorial
% Q_pX transforms from perifocal (p) to geocentric equatorial (X) frame
Q_pX = (R3_w * R1_i * R3_W)';    % Equation 4.49

% Apply transformation to get final position and velocity vectors
r = Q_pX * rp;    % Position vector in geocentric equatorial frame [km]
v = Q_pX * vp;    % Velocity vector in geocentric equatorial frame [km/s]

end