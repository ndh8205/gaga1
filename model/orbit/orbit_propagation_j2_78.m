function Xdot = orbit_propagation_j2_78(t, X, u)

    x = X(1); vx = X(2); y = X(3); vy = X(4); z = X(5); vz = X(6);
    pos = [x; y; z];

    % ff
    GM = 3.98600441500000e+05; % km^3/s^2
    RE = 6.37813630000000e+03; % km
    J2 = 0.001082627;

    r5 = norm( pos )^5;
    r2 = norm( pos )^2;
    tmp = ( pos(3)^2 ) / r2;

    % Acceleration vector due to gradient of J2 term.
    oblate_x = 1.5 * J2 * GM * ((RE^2)/r5) * pos(1) * (5 * tmp-1);
    oblate_y = 1.5 * J2 * GM * ((RE^2)/r5) * pos(2) * (5 * tmp-1);
    oblate_z = 1.5 * J2 * GM * ((RE^2)/r5) * pos(3) * (5 * tmp-3);

    x_dot = vx;
    x_2dot = ( -1 * GM * x ) / ( norm( pos ) ^ 3 ) + oblate_x + u(1);
    y_dot = vy;
    y_2dot = ( -1 * GM * y ) / ( norm( pos ) ^ 3 ) + oblate_y + u(2);
    z_dot = vz;
    z_2dot = ( -1 * GM * z ) / ( norm( pos ) ^ 3 ) + oblate_z + u(3);

    Xdot = [x_dot; x_2dot; y_dot; y_2dot; z_dot; z_2dot];
    
end