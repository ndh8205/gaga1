function coe = rv2coe(R, V, mu)
%COE_FROM_SV Convert State Vectors to Classical Orbital Elements
%
%   coe = rv2coe(R, V, mu) converts position and velocity vectors
%   to classical orbital elements in the geocentric equatorial frame.
%
%   INPUTS:
%       R   - Position vector in geocentric equatorial frame [3x1] (km)
%       V   - Velocity vector in geocentric equatorial frame [3x1] (km/s)
%       mu  - Gravitational parameter (km³/s²)
%
%   OUTPUTS:
%       coe - Classical orbital elements vector [7x1]:
%             coe(1) = h     - Specific angular momentum (km²/s)
%             coe(2) = e     - Eccentricity (dimensionless)
%             coe(3) = RA    - Right ascension of ascending node (rad)
%             coe(4) = incl  - Inclination (rad)
%             coe(5) = w     - Argument of periapsis (rad)
%             coe(6) = TA    - True anomaly (rad)
%             coe(7) = a     - Semi-major axis (km)
%
%   ALGORITHM:
%       1. Calculate basic orbital parameters
%       2. Calculate inclination and node vector
%       3. Calculate right ascension of ascending node
%       4. Calculate eccentricity vector and magnitude
%       5. Calculate argument of periapsis
%       6. Calculate true anomaly
%       7. Calculate semi-major axis
%
%   REFERENCE: Curtis, H. D. "Orbital Mechanics for Engineering Students"

% ========================================================================
% STEP 1: Calculate basic orbital parameters
% ========================================================================
eps = 1.e-12;                    % Tolerance for zero checks [-]

r = norm(R);                     % Position magnitude [km]
v = norm(V);                     % Velocity magnitude [km/s]
vr = dot(R, V) / r;              % Radial velocity component [km/s]

H = cross(R, V);                 % Angular momentum vector [km²/s]
h = norm(H);                     % Angular momentum magnitude [km²/s]

% ========================================================================
% STEP 2: Calculate inclination and node vector (Equations 4.7-4.8)
% ========================================================================
% Inclination angle from angular momentum vector
incl = acos(H(3) / h);           % Inclination [rad]

% Node vector (intersection of orbital and equatorial planes)
N = cross([0 0 1], H);           % Node vector [km²/s]
n = norm(N);                     % Node vector magnitude [km²/s]

% ========================================================================
% STEP 3: Calculate right ascension of ascending node (Equation 4.9)
% ========================================================================
if n ~= 0
    RA = acos(N(1) / n);         % Right ascension [rad]
    if N(2) < 0
        RA = 2*pi - RA;          % Quadrant check
    end
else
    RA = 0;                      % Equatorial orbit case
end

% ========================================================================
% STEP 4: Calculate eccentricity vector and magnitude (Equation 4.10)
% ========================================================================
% Eccentricity vector points toward periapsis
E = 1/mu * ((v^2 - mu/r)*R - r*vr*V);    % Eccentricity vector [-]
e = norm(E);                              % Eccentricity magnitude [-]

% ========================================================================
% STEP 5: Calculate argument of periapsis (Equation 4.12)
% ========================================================================
if n ~= 0
    if e > eps
        w = acos(dot(N, E) / n / e);      % Argument of periapsis [rad]
        if E(3) < 0
            w = 2*pi - w;                  % Quadrant check
        end
    else
        w = 0;                             % Circular orbit case
    end
else
    w = 0;                                 % Equatorial orbit case
end

% ========================================================================
% STEP 6: Calculate true anomaly (Equation 4.13a)
% ========================================================================
if e > eps
    TA = acos(dot(E, R) / e / r);         % True anomaly [rad]
    if vr < 0
        TA = 2*pi - TA;                    % Quadrant check
    end
else
    % Circular orbit case - measure angle from ascending node
    cp = cross(N, R);                      % Cross product for quadrant check
    if cp(3) >= 0
        TA = acos(dot(N, R) / n / r);      % True anomaly [rad]
    else
        TA = 2*pi - acos(dot(N, R) / n / r);
    end
end

% ========================================================================
% STEP 7: Calculate semi-major axis (Equation 4.62)
% ========================================================================
% Note: a < 0 for hyperbolic orbits
a = h^2 / mu / (1 - e^2);                  % Semi-major axis [km]

% ========================================================================
% STEP 8: Assemble output vector
% ========================================================================
coe = [h e RA incl w TA a];

end