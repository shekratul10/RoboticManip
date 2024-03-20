function task_handler(task, port_num)
    ID_BASE                     = 11;
    ID_1                        = 12;
    ID_2                        = 13;
    ID_3                        = 14;
    ID_CLAW                     = 15;

    PROTOCOL_VERSION            = 2.0;
    ADDR_PRO_GOAL_POSITION       = 116; 
    
    PAUSE_PRE_CLAW = 0.25;
    PAUSE_POST_CLAW = 0.25;
    PAUSE_POST_MOVEMENT = 0.25;
    
    if task(1) == 0
        [x, y, z, gamma] = read_curr_pos(port_num);
        % disp([x, y, z, gamma]);
        % disp(IK(x, y, z, gamma));
        disp(task);
        % disp(IK(task(2), task(3), task(4), task(5)));
        disp("=====")
        interpolated_points = cubic_spline_interpolation(IK(x, y, z, gamma), IK(task(2), task(3), task(4), task(5)), 5, 0.1);
        for j=1:size(interpolated_points, 1)
            theta = interpolated_points(j, :);
            % disp(theta);
            write4ByteTxRx(port_num, PROTOCOL_VERSION, ID_BASE, ADDR_PRO_GOAL_POSITION, mapping_angle('tbase', theta(1)));
            write4ByteTxRx(port_num, PROTOCOL_VERSION, ID_1, ADDR_PRO_GOAL_POSITION, mapping_angle('t1', theta(2)));
            write4ByteTxRx(port_num, PROTOCOL_VERSION, ID_2, ADDR_PRO_GOAL_POSITION, mapping_angle('t2', theta(3)));
            write4ByteTxRx(port_num, PROTOCOL_VERSION, ID_3, ADDR_PRO_GOAL_POSITION, mapping_angle('t3', theta(4)));
        end
        pause(PAUSE_POST_MOVEMENT);

    elseif task(1) == 1
        pause(PAUSE_PRE_CLAW);
        fprintf('Moving Claw Position to %d\n', task(2));
        write4ByteTxRx(port_num, PROTOCOL_VERSION, ID_CLAW, ADDR_PRO_GOAL_POSITION, task(2));
        pause(PAUSE_POST_CLAW);
    
    elseif task(1) == 2
        disp("no interpolation to")
        disp(task);
        theta = IK(task(2), task(3), task(4), task(5));
        write4ByteTxRx(port_num, PROTOCOL_VERSION, ID_BASE, ADDR_PRO_GOAL_POSITION, mapping_angle('tbase', theta(1)));
        write4ByteTxRx(port_num, PROTOCOL_VERSION, ID_1, ADDR_PRO_GOAL_POSITION, mapping_angle('t1', theta(2)));
        write4ByteTxRx(port_num, PROTOCOL_VERSION, ID_2, ADDR_PRO_GOAL_POSITION, mapping_angle('t2', theta(3)));
        write4ByteTxRx(port_num, PROTOCOL_VERSION, ID_3, ADDR_PRO_GOAL_POSITION, mapping_angle('t3', theta(4)));

    elseif task(1) == 3
        pause(task(2));
    end
end