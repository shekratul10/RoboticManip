function task_list = linear_interpolation(coord1, coord2, num_points, gamma)

    x_values = linspace(coord1(1), coord2(1), num_points)';
    y_values = linspace(coord1(2), coord2(2), num_points)';
    z_values = linspace(coord1(3), coord2(3), num_points)';
    zero_list = zeros(num_points, 1);
    twos_list = 2 * ones(num_points, 1);
    gamma_list = gamma * ones(num_points, 1);

    % Combine x, y, and z values into a matrix of points
    task_list = [twos_list, x_values, y_values, z_values, gamma_list]; % Each row represents a point [x, y, z]

end