function output = mapping_angle(theta, angle)
    if strcmpi(theta, 'tbase')
        min_angle = -pi/2;
        max_angle = pi/2;
        min_val = 90/0.088;
        max_val = 270/0.088;
        output = ((angle - min_angle) / (max_angle - min_angle)) * (max_val - min_val) + min_val;

    elseif strcmpi(theta, 't1')
        % 0 is up right, +pi/2 is all the way back, -pi/2 is all the way forward
        min_angle = -pi/2;
        max_angle = pi/2;
        min_val = 270/0.088;
        max_val = 90/0.088;
        output = ((angle - min_angle) / (max_angle - min_angle)) * (max_val - min_val) + min_val;

    elseif strcmpi(theta, 't2')
        min_angle = -pi/2;
        max_angle = pi/2;
        min_val = 270/0.088;
        max_val = 90/0.088;
        output = ((angle - min_angle) / (max_angle - min_angle)) * (max_val - min_val) + min_val;

    elseif strcmpi(theta, 't3')
        min_angle = -pi/2;
        max_angle = pi/2;
        max_val = 90/0.088;
        min_val = 270/0.088;
        output = ((angle - min_angle) / (max_angle - min_angle)) * (max_val - min_val) + min_val;
   else
        error('Invalid joint');
    end
end