function [x, y, z, angle] = end_effector_state(T3,T4)
    end_effector = (T4(1:3, 4) - T3(1:3, 4))';
    r = sqrt(end_effector(1)^2 + end_effector(2)^2);
    height = end_effector(3);
    angle = atan2(height, r);
    x = T4(1, 4);
    y = T4(2, 4);
    z = T4(3, 4);
end