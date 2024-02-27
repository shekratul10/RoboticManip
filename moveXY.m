function moveXY(theta)

%% 
% ----- SET MOTION LIMITS ----------- %
ADDR_MAX_POS = 48;
ADDR_MIN_POS = 52;
% Set max position limit
write4ByteTxRx(port_num, PROTOCOL_VERSION, 11, ADDR_MAX_POS, DXL_MAXIMUM_POSITION_VALUE);
% Set min position limit
write4ByteTxRx(port_num, PROTOCOL_VERSION, 11, ADDR_MIN_POS, DXL_MINIMUM_POSITION_VALUE);
% ---------------------------------- %

% Put actuator into Position Control Mode
write1ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID, ADDR_PRO_OPERATING_MODE, 3);

% Enable Dynamixel Torque
write1ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID, ADDR_PRO_TORQUE_ENABLE, TORQUE_ENABLE);

% Control loop

target_position = (theta/360) * 4095; % Target position to move to
write4ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID, ADDR_PRO_GOAL_POSITION, target_position);

% Wait for the movement to complete
while true
    % Read present position
    dxl_present_position = read4ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID, ADDR_PRO_PRESENT_POSITION);
    
    % Check if the servo has reached the target position
    if abs(target_position - dxl_present_position) < 10  % You may need to adjust the threshold value
        fprintf('Dynamixel has reached the target position: %d\n', dxl_present_position);
        break;
    end
end

% Disable Dynamixel Torque
write1ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID, ADDR_PRO_TORQUE_ENABLE, TORQUE_DISABLE);



end