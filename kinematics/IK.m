function output = IK(x, y, z, gamma)
    % constants
    L1 = 0.130;
    L2 = 0.124;
    L3 = 0.126;

    t1 = atan2(y, x);

    r = sqrt(x^2 + y^2);
    height = z - 0.077;
    
    x = r - L3 * cos(gamma);
    y = height - L3 * sin(gamma);
    [t2, t3] = two_link_planar_ik(x, y, L1, L2);
    t4 = -t2 - t3 + gamma;
    
    t2 = t2 - (pi/2.0 - atan(0.024/0.128));
    t3 = t3 + (pi/2.0 - atan(0.024/0.128));
    % t4 = -t4;
    output = [t1, t2, t3, t4];
end