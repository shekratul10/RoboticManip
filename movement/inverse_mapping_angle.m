function angle = inverse_mapping_angle(theta, value)
    if strcmpi(theta, 'tbase')
        min_angle = -pi/2;
        max_angle = pi/2;
        min_val = 90/0.088;
        max_val = 270/0.088;
        angle = ((value - min_val) / (max_val - min_val)) * (max_angle - min_angle) + min_angle;

    elseif strcmpi(theta, 't1')
        min_angle = -pi/2;
        max_angle = pi/2;
        min_val = 270/0.088;
        max_val = 90/0.088;
        angle = ((value - min_val) / (max_val - min_val)) * (max_angle - min_angle) + min_angle;

    elseif strcmpi(theta, 't2')
        min_angle = -pi/2;
        max_angle = pi/2;
        min_val = 270/0.088;
        max_val = 90/0.088;
        angle = ((value - min_val) / (max_val - min_val)) * (max_angle - min_angle) + min_angle;

    elseif strcmpi(theta, 't3')
        min_angle = -pi/2;
        max_angle = pi/2;
        max_val = 90/0.088;
        min_val = 270/0.088;
        angle = ((value - min_val) / (max_val - min_val)) * (max_angle - min_angle) + min_angle;
    else
        error('Invalid joint');
    end
end
