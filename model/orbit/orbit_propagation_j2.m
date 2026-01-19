function xdot = orbit_propagation_j2( t, X, u_I, w_I, param )
    
    r = X(1:3);
    v = X(4:6);

    % ff
    Mu = param.orbit.Mu;
    R_e = param.orbit.R_e;
    J2 = param.orbit.J2;
    
    r3 = norm( r )^3;
    r5 = norm( r )^5;
    
    rho_1 = 1 - 5 * ( (r(3) / norm( r )).^2 );
    rho_2 = rho_1;
    rho_3 = 3 - 5 * ( (r(3) / norm( r )).^2 );
    rho_mat = diag( [ rho_1 ; rho_2; rho_3 ] );
    
    r1dot = v;
    r2dot = -Mu/r3 * r - 1.5 * J2 * Mu * ( R_e^2 / r5 ) * rho_mat * r + u_I + w_I;
    xdot = [ r1dot; r2dot ];
    
end