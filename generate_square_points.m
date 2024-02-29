function output = generate_square_points(plane, res, c1, c2, offset)
    x = linspace(offset, offset, res*4);
    y = linspace(offset, offset, res*4);
    z = linspace(offset, offset, res*4);

    % Generate points for the selected plane
    if strcmpi(plane, 'xy')
        % Square in xy plane
        x(1:res) = linspace(c1(1), c2(1), res);
        y(1:res) = linspace(c1(2), c1(2), res);
        
        x(res+1:2*res) = linspace(c2(1), c2(1), res);
        y(res+1:2*res) = linspace(c1(2), c2(2), res);
        
        x(2*res+1:3*res) = linspace(c2(1), c1(1), res);
        y(2*res+1:3*res) = linspace(c2(2), c2(2), res);
        
        x(3*res+1:end) = linspace(c1(1), c1(1), res);
        y(3*res+1:end) = linspace(c2(2), c1(2), res);

    elseif strcmpi(plane, 'xz')
        % Square in xz plane
        x(1:res) = linspace(c1(1), c2(1), res);
        z(1:res) = linspace(c1(2), c1(2), res);
        
        x(res+1:2*res) = linspace(c2(1), c2(1), res);
        z(res+1:2*res) = linspace(c1(2), c2(2), res);
        
        x(2*res+1:3*res) = linspace(c2(1), c1(1), res);
        z(2*res+1:3*res) = linspace(c2(2), c2(2), res);
        
        x(3*res+1:end) = linspace(c1(1), c1(1), res);
        z(3*res+1:end) = linspace(c2(2), c1(2), res);

    elseif strcmpi(plane, 'yz')
        % Square in yz plane
        y(1:res) = linspace(c1(1), c2(1), res);
        z(1:res) = linspace(c1(2), c1(2), res);
        
        y(res+1:2*res) = linspace(c2(1), c2(1), res);
        z(res+1:2*res) = linspace(c1(2), c2(2), res);
        
        y(2*res+1:3*res) = linspace(c2(1), c1(1), res);
        z(2*res+1:3*res) = linspace(c2(2), c2(2), res);
        
        y(3*res+1:end) = linspace(c1(1), c1(1), res);
        z(3*res+1:end) = linspace(c2(2), c1(2), res);

    else
        error('Invalid plane. Choose either "xy", "xz", or "yz".');
    end
    output = [x; y; z];
    % disp(output);
end