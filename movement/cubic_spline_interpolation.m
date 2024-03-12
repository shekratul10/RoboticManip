function interpolated_points = cubic_spline_interpolation(theta1, theta2, num_waypoints, resolution)
    interpolated_points = zeros(1/resolution + 1, 4);
    for i=1:4
        waypoints = linspace(theta1(i), theta2(i), num_waypoints);
        t = linspace(0, 1, num_waypoints);
        splines = spline(t, [theta1(i), waypoints, theta2(i)]);
    
        t_interpolated = 0:resolution:1;
        interpolated = ppval(splines, t_interpolated);
        interpolated_points(:, i) = interpolated;
    end
end