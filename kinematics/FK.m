function [T0, T1, T2, T3, T4] = FK(joint_angles)
    beta = pi/2 - atan(0.024/0.128);
    T0 = dhparam2matrix(0,0,0,joint_angles(1));
    T1 = T0 * dhparam2matrix(0, 0, 0.077,0) * dhparam2matrix(0, pi/2, 0,joint_angles(2) +beta);
    T2 = T1 * dhparam2matrix(0.130, 0, 0, joint_angles(3)-beta);
    T3 = T2 * dhparam2matrix(0.124, 0, 0, joint_angles(4));
    T4 = T3 * dhparam2matrix(0.126, 0, 0, 0);
end