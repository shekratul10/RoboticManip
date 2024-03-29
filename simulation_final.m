% Plot parameters
fig = figure;
hold on;
grid on;
view(60, 20);
axis equal;
axis([-0.2 0.2 -0.1 0.3 -0.1 0.3]);


% Base Robot

%theta = IK(0.15, 0, 0.15, 0);
%plot_robot(theta);


% theta = IK(0.15, 0, 0.05, -pi/2);
% plot_robot(theta);

% plot_robot([0,10 * 0.0174533,0,-70/0.0174533]);
points = generate_square_points('yz', 30, [0.05,0.05], [0.15,0.15], 0);

for i = 1:length(points)
    theta = IK(points(1, i), points(2, i), points(3, i), -pi/2);
    links = plot_robot(theta);
    frames = plot_frames(theta);
    pause(0.01);
    if i ~= length(points)
        delete(links);
        delete(frames);
    end

end

% Plot robot
function link_array = plot_robot(joint_angles)
    [T0, T1, T2, T3, T4] = FK(joint_angles);
    link1 = plot_link(T0, T0, 'k', 5);
    link2 = plot_link(T0, T1, 'k', 4);
    link3 = plot_link(T1, T2, 'k', 3);
    link4 = plot_link(T2, T3, 'k', 2);
    link5 = plot_link(T3, T4, 'k', 1);
    
    tip_pos = T4(:,4);
    plot3(tip_pos(1), tip_pos(2), tip_pos(3), 'o', 'MarkerSize', 4, 'MarkerFaceColor', 'r', 'MarkerEdgeColor', 'b');
    
    link_array = [link1, link2, link3, link4, link5];


end

% Plot robot
function frame_array = plot_frames(joint_angles)
    [T0, T1, T2, T3, T4] = FK(joint_angles);
    frame1 = plot_frame(T0, T0, 'k', 1);
    frame2 = plot_frame(T0, T1, 'k', 1);
    frame3 = plot_frame(T1, T2, 'k', 1);
    frame4 = plot_frame(T2, T3, 'k', 1);
    frame5 = plot_frame(T3, T4, 'k', 1);
    
    
    frame_array = [frame1, frame2, frame3, frame4, frame5];


end

function link_output = plot_link(Ti_1, T, color, thickness)
    x = T(1:3, 1)/20;
    y = T(1:3, 2)/20;
    z = T(1:3, 3)/20;
    prev_pos = Ti_1(:,4);
    curr_pos = T(:,4);

    % Plot link
    link_output = plot3([prev_pos(1), curr_pos(1)], [prev_pos(2), curr_pos(2)], [prev_pos(3), curr_pos(3)], 'Color',color,'LineWidth', thickness);
end

function frame_output = plot_frame(Ti_1, T, color, thickness)
    x = T(1:3, 1)/20;
    y = T(1:3, 2)/20;
    z = T(1:3, 3)/20;
    prev_pos = Ti_1(:,4);
    curr_pos = T(:,4);


    % plot3([prev_pos(1), curr_pos(1)], [prev_pos(2), curr_pos(2)], [prev_pos(3), curr_pos(3)], 'Color',color,'LineWidth', thickness);

    % Plot frames
     red = plot3([curr_pos(1), curr_pos(1) + x(1)], [curr_pos(2), curr_pos(2) + x(2)], [curr_pos(3), curr_pos(3) + x(3)], 'r', 'LineWidth', thickness);
     green = plot3([curr_pos(1), curr_pos(1) + y(1)], [curr_pos(2), curr_pos(2) + y(2)], [curr_pos(3), curr_pos(3) + y(3)], 'g', 'LineWidth', thickness);
     blue = plot3([curr_pos(1), curr_pos(1) + z(1)], [curr_pos(2), curr_pos(2) + z(2)], [curr_pos(3), curr_pos(3) + z(3)], 'b', 'LineWidth', thickness);

    % Plot link
    frame_output = [red, green, blue];
end
