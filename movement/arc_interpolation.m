function task_list = arc_interpolation(num_points, offset)
    radius = 0.04;
    center = [0.2, 0.1, offset];
    
    theta = linspace(-pi/2, pi/2, num_points); % Angle ranging from 0 to pi for semicircle
    x = center(1) - radius * cos(theta);
    y = center(2) + radius * sin(theta);
    z = center(3) * ones(size(x)); % Keep z constant for xy plane
    zero_list = zeros(size(x));
    twos_list = 2 * ones(size(x));

    task_list = [twos_list; x; y; z; zero_list]';
end