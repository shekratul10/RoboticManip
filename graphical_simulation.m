% Plot parameters
fig = figure;
hold on;
grid on;
view(3);
axis equal;
axis([-0.5 0.5 -0.5 0.5 -0.1 0.5]);

global L1;
global L2;
global L3;

L1 = 0.130;
L2 = 0.124;
L3 = 0.126;


points = generate_square_points('xy', 5, [-0.1,-0.1], [-0.2,-0.2], 0.077);

for i = 1:length(points)
    plot3(points(1,i), points(2,i), points(3,i), 'o', 'MarkerSize', 4, 'MarkerFaceColor', 'green', 'MarkerEdgeColor', 'black')
end

% Base Robot
joint_angles = [pi/2,10 * 0.0174533,0,-70/0.0174533];
plot_robot(joint_angles);

thetas = inverse_kinematics(0.2, 0, 0.03, 0.130, 0.124, 0.126, -pi/2);
display(thetas);
plot_robot(thetas);


% for i = 1:length(points)
%     x = points(1,i);
%     y = points(2,i);
%     z = points(3,i);
% 
%     if norm([x, y, z+0.077]) > L1 + L2 + L3
%         disp([num2str(x), ', ', num2str(y), ', ', num2str(z), ' is ', num2str(norm([x, y, z+0.077])), ' away']);
%         max_reach = L1 + L2 + L3;
%         disp(['Exceeds maximum reach of ', num2str(max_reach)]);
%     else
%         thetas = inverse_kinematics(x, y, z, 0.130, 0.124, 0.126);
%         plot_robot(thetas);
%     end
% 
%     pause(0.1);
% end

% Plot robot
function plot_robot(joint_angles)
    % DH values
    beta = pi/2 - atan2(0.024, 0.128);
    a = [0, 0.130, 0.124, 0.126]; % Link lengths (meters)
    alpha = [pi/2, 0, 0, 0]; % Link twists (radians)
    d = [0.077, 0, 0, 0]; % Link offsets (meters)
    theta = [0, beta, -beta, 0]; % Joint angles (radians)
    theta = theta + joint_angles;

    % Base Frame
    T0 = dhparam2matrix(0,0,0,0);
    plot_link(T0, T0, 'k', 1);

    % x0 = T0(1:3,1)/20;
    % y0 = T0(1:3,2)/20;
    % z0 = T0(1:3,3)/20;
    % plot3([0, x0(1)], [0, x0(2)], [0, x0(3)], 'Color','k','LineWidth',1);
    % plot3([0, y0(1)], [0, y0(2)], [0, y0(3)], 'Color','k','LineWidth',1);
    % plot3([0, z0(1)], [0, z0(2)], [0, z0(3)], 'Color','k','LineWidth',1);

    for i = 1:length(4)
        % Compute transformation matrix
        T1 = T0 * dhparam2matrix(a(1), alpha(1), d(1), theta(1));
        T2 = T1 * dhparam2matrix(a(2), alpha(2), d(2), theta(2));
        T3 = T2 * dhparam2matrix(a(3), alpha(3), d(3), theta(3));
        T4 = T3 * dhparam2matrix(a(4), alpha(4), d(4), theta(4));
        
        plot_link(T0, T1, 'k', 1);
        plot_link(T1, T2, 'k', 1);
        plot_link(T2, T3, 'k', 1);
        plot_link(T3, T4, 'k', 1);
        
        % Plot tip
        tip_pos = T4(:,4);
        plot3(tip_pos(1), tip_pos(2), tip_pos(3), 'o', 'MarkerSize', 4, 'MarkerFaceColor', 'r', 'MarkerEdgeColor', 'b')
    end
end

function plot_link(Ti_1, T, color, thickness)
    % Extract orientation and position from transformation matrix
    x = T(1:3, 1)/20;
    y = T(1:3, 2)/20;
    z = T(1:3, 3)/20;
    prev_pos = Ti_1(:,4);
    curr_pos = T(:,4);

    % Plot link
    plot3([prev_pos(1), curr_pos(1)], [prev_pos(2), curr_pos(2)], [prev_pos(3), curr_pos(3)], 'Color',color,'LineWidth', thickness);

    % Plot frames
    % frame_color = 'blue'
    % plot3([curr_pos(1), curr_pos(1) + x(1)], [curr_pos(2), curr_pos(2) + x(2)], [curr_pos(3), curr_pos(3) + x(3)], frame_color, 'LineWidth', thickness);
    % plot3([curr_pos(1), curr_pos(1) + y(1)], [curr_pos(2), curr_pos(2) + y(2)], [curr_pos(3), curr_pos(3) + y(3)], frame_color, 'LineWidth', thickness);
    % plot3([curr_pos(1), curr_pos(1) + z(1)], [curr_pos(2), curr_pos(2) + z(2)], [curr_pos(3), curr_pos(3) + z(3)], frame_color, 'LineWidth', thickness);
end

function T = dhparam2matrix(a, alpha, d, theta)
    T = [cos(theta), -sin(theta)*cos(alpha), sin(theta)*sin(alpha), a*cos(theta);
         sin(theta), cos(theta)*cos(alpha), -cos(theta)*sin(alpha), a*sin(theta);
         0, sin(alpha), cos(alpha), d;
         0, 0, 0, 1];
end


function theta = inverse_kinematics(x, y, z, L1, L2, L3, input)

    % calculates t1 between - pi/2 and + pi / 2 given x and y coords
    t1 = atan2(y, x);
    t1 = mod(t1 + pi, 2*pi) - pi;

    % assume L3 is directly down and accounts for offset
    height = z - 0.077;
    r = sqrt(x^2 + y^2);

    if input == 1000
        gamma = -pi/2;
        x = r - L3 * cos(gamma);
        y = height - L3 * sin(gamma);
        if norm([x, y]) > L1 + L2
            angle = atan(height/r);
        else
            angle = (- pi/2 + atan2(height, r)) / 2;
        end
        gamma = angle;
    else
        gamma = input;
    end
    
    % gamma: angle of last link with respect to the horizontal axis
    % from + pi/2 straight up to -pi / 2 straight down

    output = three_link_planar_ik(r, height, gamma, L1, L2, L3);

    theta = [t1; output];
end

function theta = three_link_planar_ik(x, y, gamma, L1, L2, L3)
    x = x - L3 * cos(gamma);
    y = y - L3 * sin(gamma);
    
    [t1, t2] = two_link_planar_ik(x, y, L1, L2);
    t3 = - t1 - t2 + gamma;
    theta = [t1; t2; t3];
end


function [t1, t2] = two_link_planar_ik(x, y, L1, L2)
    if norm([x, y]) > L1 + L2
        error('Two Link: Position out of reachable workspace');
    end
    
    % elbow up config, to change to elbow down, remove negative sign
    t2 = - acos((x^2 + y^2 - L1^2 - L2^2) / (2 * L1 * L2));
    t1 = atan2(y, x) - atan2(L2 * sin(t2), L1 + L2 * cos(t2));
end