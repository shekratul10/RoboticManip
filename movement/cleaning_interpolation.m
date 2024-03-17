function task_list = cleaning_interpolation(coord1, coord2, gamma)
    x1 = coord1(1);
    y1 = coord1(2);
    x2 = coord2(1);
    y2 = coord2(2);

    % Change stepsize
    stepsize = 0.0025;
    
    num_x = ceil((max(x1, x2) - min(x1, x2)) / stepsize) + 1;
    num_y = ceil((max(y1, y2) - min(y1, y2)) / stepsize) + 1;

    x_arr = linspace(x1, x2, num_x);
    flipped_x = fliplr(x_arr);
    y_arr = linspace(y1, y2, num_y);
    
    xy_coord_arr = [];  % Initialize output_arr
    
    for i=1:num_y
        curr_y = y_arr(i) * ones(num_x, 1);
        if mod(i, 2) ~= 0
            xy_coord_arr = [xy_coord_arr; [x_arr', curr_y]]; % Concatenate x_arr and curr_y vertically
        else
            xy_coord_arr = [xy_coord_arr; [flipped_x', curr_y]]; % Concatenate flipped_x and curr_y vertically
        end
    end

    z_values = linspace(coord1(3), coord2(3), length(xy_coord_arr))';
    twos_list = 2 * ones(length(xy_coord_arr), 1);
    gamma_list = gamma * ones(length(xy_coord_arr), 1);

    task_list = [twos_list, xy_coord_arr, z_values, gamma_list];
end