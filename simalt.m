function simalt()
    % Plot parameters
    fig = figure;
    hold on;
    grid on;
    view(45, 30);
    axis equal;
    axis([-0.2 0.2 -0.1 0.3 -0.1 0.22]);

    % Base Robot

    % theta = IK(0.15, 0, 0.15, 0);
    % plot_robot(theta);


    % theta = IK(0.15, 0, 0.05, -pi/2);
    % plot_robot(theta);

    % plot_robot([0,10 * 0.0174533,0,-70/0.0174533]);
    points = generate_square_points('xy', 30, [0.05,0.05], [0.15,0.15], 0);
    
    % Initialize variable to store the last end effector position
    T4_last = [];

    for i = 1:length(points)
        theta = IK(points(1, i), points(2, i), points(3, i), -pi/2);
        links = plot_robot(theta, T4_last); % Pass T4_last as an argument
        pause(0.01);
        if i ~= length(points)
            delete(links);
        end
        T4_last = FK(theta); % Update T4_last for the next iteration
    end
end

function link_array = plot_robot(joint_angles, T4_last)
    [T0, T1, T2, T3, T4] = FK(joint_angles);
    link1 = plot_link(T0, T0, 'k', 1);
    link2 = plot_link(T0, T1, 'k', 1);
    link3 = plot_link(T1, T2, 'k', 1);
    link4 = plot_link(T2, T3, 'k', 1);
    link5 = plot_link(T3, T4, 'k', 1);
    
    tip_pos = T4(:,4);
    plot3(tip_pos(1), tip_pos(2), tip_pos(3), 'o', 'MarkerSize', 4, 'MarkerFaceColor', 'r', 'MarkerEdgeColor', 'b');
    
    % Delete frames from previous iterations
    delete(findobj(gca, 'Tag', 'frame'));
    
    % Plot frames only for the last iteration
    if ~isempty(T4_last) && ~isequal(tip_pos, T4_last(1:3, 4))
        x = T4(1:3, 1)/20;
        y = T4(1:3, 2)/20;
        z = T4(1:3, 3)/20;
        curr_pos = T4(:, 4);
        plot3([curr_pos(1), curr_pos(1) + x(1)], [curr_pos(2), curr_pos(2) + x(2)], [curr_pos(3), curr_pos(3) + x(3)], 'r', 'LineWidth', 1, 'Tag', 'frame');
        plot3([curr_pos(1), curr_pos(1) + y(1)], [curr_pos(2), curr_pos(2) + y(2)], [curr_pos(3), curr_pos(3) + y(3)], 'g', 'LineWidth', 1, 'Tag', 'frame');
        plot3([curr_pos(1), curr_pos(1) + z(1)], [curr_pos(2), curr_pos(2) + z(2)], [curr_pos(3), curr_pos(3) + z(3)], 'b', 'LineWidth', 1, 'Tag', 'frame');
    end
    
    link_array = [link1, link2, link3, link4, link5];
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
