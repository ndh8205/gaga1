function a_grav = orbit_gravity_j2(pos, GM, RE, J2)
    r_mag = norm(pos);
    a_2body = -GM / r_mag^3 * pos;
    
    z = pos(3);
    r5 = r_mag^5;
    z2_r2 = (z^2) / (r_mag^2);
    
    a_J2 = -1.5 * J2 * GM * RE^2 / r5 * ...
           [(1 - 5*z2_r2) * pos(1);
            (1 - 5*z2_r2) * pos(2);
            (3 - 5*z2_r2) * pos(3)];
    
    a_grav = a_2body + a_J2;
end