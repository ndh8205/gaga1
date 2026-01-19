function Phi = getCWMatrix(n, dt)
    % getCWMatrix - Clohessy-Wiltshire State Transition Matrix 계산
    % Inputs:
    %   n  - mean motion [rad/s]
    %   dt - time step [s]
    % Output:
    %   Phi - 6x6 State Transition Matrix
    
    nt = n * dt;
    
    % Position transition matrix (Φrr) - 3x3
    Phi_rr = [4-3*cos(nt),     0,  0;
              6*(sin(nt)-nt),   1,  0;
              0,                 0,  cos(nt)];
    
    % Position-velocity transition matrix (Φrv) - 3x3
    Phi_rv = [sin(nt)/n,            2*(1-cos(nt))/n,        0;
              2*(cos(nt)-1)/n,       (4*sin(nt)-3*nt)/n,     0;
              0,                     0,                       sin(nt)/n];
    
    % Velocity-position transition matrix (Φvr) - 3x3
    Phi_vr = [3*n*sin(nt),      0,  0;
              6*n*(cos(nt)-1),   0,  0;
              0,                 0,  -n*sin(nt)];
    
    % Velocity transition matrix (Φvv) - 3x3
    Phi_vv = [cos(nt),      2*sin(nt),      0;
              -2*sin(nt),   4*cos(nt)-3,    0;
              0,            0,              cos(nt)];
    
    % Complete 6x6 State Transition Matrix
    Phi = [Phi_rr,  Phi_rv;
           Phi_vr,  Phi_vv];
end