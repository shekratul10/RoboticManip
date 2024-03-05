function [t1, t2] = two_link_planar_ik(x, y, L1, L2)
    if norm([x, y]) > L1 + L2
        error('Two Link: Position out of reachable workspace');
    end
    
    % elbow up config, to change to elbow down, remove negative sign
    t2 = - acos((x^2 + y^2 - L1^2 - L2^2) / (2 * L1 * L2));
    t1 = atan2(y, x) - atan2(L2 * sin(t2), L1 + L2 * cos(t2));
end