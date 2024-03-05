function T = dhparam2matrix(a, alpha, d, theta)
    % T = [cos(theta), -sin(theta)*cos(alpha), sin(theta)*sin(alpha), a*cos(theta);
    %      sin(theta), cos(theta)*cos(alpha), -cos(theta)*sin(alpha), a*sin(theta);
    %      0, sin(alpha), cos(alpha), d;
    %      0, 0, 0, 1];

    T =  [cos(theta)               -sin(theta)             0               a;
        sin(theta)*cos(alpha)   cos(theta)*cos(alpha)   -sin(alpha)     -sin(alpha)*d;
        sin(theta)*sin(alpha)   cos(theta)*sin(alpha)   cos(alpha)      cos(alpha)*d;
        0                       0                       0               1];
end