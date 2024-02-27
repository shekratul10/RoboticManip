% Read the position of the dynamixel horn with the torque off
% The code executes for a given amount of time then terminates

clc;
clear all;

lib_name = '';

% Determine the appropriate library based on the operating system
if strcmp(computer, 'PCWIN')
  lib_name = 'dxl_x86_c';
elseif strcmp(computer, 'PCWIN64')
  lib_name = 'dxl_x64_c';
elseif strcmp(computer, 'GLNX86')
  lib_name = 'libdxl_x86_c';
elseif strcmp(computer, 'GLNXA64')
  lib_name = 'libdxl_x64_c';
elseif strcmp(computer, 'MACI64')
  lib_name = 'libdxl_mac_c';
end

% Load Libraries
if ~libisloaded(lib_name)
    [notfound, warnings] = loadlibrary(lib_name, 'dynamixel_sdk.h', 'addheader', 'port_handler.h', 'addheader', 'packet_handler.h');
end

%% ---- Control Table Addresses ---- %%

ADDR_PRO_TORQUE_ENABLE       = 64;           % Control table address is different in Dynamixel model
ADDR_PRO_GOAL_POSITION       = 116; 
ADDR_PRO_PRESENT_POSITION    = 132; 
ADDR_PRO_OPERATING_MODE      = 11;

%% ---- Other Settings ---- %%

% Protocol version
PROTOCOL_VERSION            = 2.0;          % See which protocol version is used in the Dynamixel

% Default setting
DXL_ID                      = 11;            % Dynamixel ID: 1
BAUDRATE                    = 115200;
DEVICENAME                  = 'COM7';       % Check which port is being used on your controller
                                            
TORQUE_ENABLE               = 1;            % Value for enabling the torque
TORQUE_DISABLE              = 0;            % Value for disabling the torque

% Position limits
DXL_MINIMUM_POSITION_VALUE  = 700;      % Dynamixel minimum position value
DXL_MAXIMUM_POSITION_VALUE  = 4095;   % Dynamixel maximum position value

% Communication status
COMM_SUCCESS                = 0;            % Communication Success result value
COMM_TX_FAIL                = -1001;        % Communication Tx Failed

%% ------------------ %%

% Initialize PortHandler Structs
port_num = portHandler(DEVICENAME);

% Initialize PacketHandler Structs
packetHandler();

% Open port
if (openPort(port_num))
    fprintf('Port Open\n');
else
    fprintf('Failed to open the port\n');
    unloadlibrary(lib_name);
    return;
end

% Set port baudrate
if (setBaudRate(port_num, BAUDRATE))
    fprintf('Baudrate Set\n');
else
    fprintf('Failed to change the baudrate!\n');
    closePort(port_num);
    unloadlibrary(lib_name);
    return;
end

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


% Close port
closePort(port_num);
fprintf('Port Closed\n');

% Unload Library
unloadlibrary(lib_name);
