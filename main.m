% Read the position of the dynamixel horn with the torque off
% The code executes for a given amount of time then terminates


clc;
clear all;

lib_name = '';

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
ADDR_PRO_DRIVE_MODE          = 10;

%% ---- Other Settings ---- %%

% Protocol version
PROTOCOL_VERSION            = 2.0;          % See which protocol version is used in the Dynamixel

% Default setting

ID_BASE                     = 11;
ID_1                        = 12;
ID_2                        = 13;
ID_3                        = 14;
ID_CLAW                     = 15;

BAUDRATE                    = 115200;
DEVICENAME                  = 'COM6';
                                            
TORQUE_ENABLE               = 1;            % Value for enabling the torque
TORQUE_DISABLE              = 0;            % Value for disabling the torque
DXL_MINIMUM_POSITION_VALUE  = -150000;      % Dynamixel will rotate between this value
DXL_MAXIMUM_POSITION_VALUE  = 150000;       % and this value (note that the Dynamixel would not move when the position value is out of movable range. Check e-manual about the range of the Dynamixel you use.)
DXL_MOVING_STATUS_THRESHOLD = 20;           % Dynamixel moving status threshold

ESC_CHARACTER               = 'e';          % Key for escaping loop

COMM_SUCCESS                = 0;            % Communication Success result value
COMM_TX_FAIL                = -1001;        % Communication Tx Failed

%% ------------------ %%

% Initialize PortHandler Structs
% Set the port path
% Get methods and members of PortHandlerLinux or PortHandlerWindows
port_num = portHandler(DEVICENAME);

% Initialize PacketHandler Structs
packetHandler();

index = 1;
dxl_comm_result = COMM_TX_FAIL;           % Communication result
dxl_goal_position = [DXL_MINIMUM_POSITION_VALUE DXL_MAXIMUM_POSITION_VALUE];         % Goal position

dxl_error = 0;                              % Dynamixel error
dxl_present_position = 0;                   % Present position


% Open port
if (openPort(port_num))
    fprintf('Port Open\n');
else
    unloadlibrary(lib_name);
    fprintf('Failed to open the port\n');
    input('Press any key to terminate...\n');
    return;
end


% Set port baudrate
if (setBaudRate(port_num, BAUDRATE))
    fprintf('Baudrate Set\n');
else
    unloadlibrary(lib_name);
    fprintf('Failed to change the baudrate!\n');
    input('Press any key to terminate...\n');
    return;
end

%% ----- SET MOTION LIMITS ----------- %%

ADDR_MAX_POS = 48;
ADDR_MIN_POS = 52;
MAX_POS = 3400;
MIN_POS = 600;
% % Set max position limit
% write4ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID1, ADDR_MAX_POS, MAX_POS);
% write4ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID2, ADDR_MAX_POS, MAX_POS);
% % Set min position limit
% write4ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID1, ADDR_MIN_POS, MIN_POS);
% write4ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID2, ADDR_MIN_POS, MIN_POS);

%% ---------------------------------- %%

% Put actuator into Position Control Mode
write1ByteTxRx(port_num, PROTOCOL_VERSION, ID_BASE, ADDR_PRO_OPERATING_MODE, 3);

% Change Drive Mode and Profile
write4ByteTxRx(port_num, PROTOCOL_VERSION, ID_BASE, ADDR_PRO_DRIVE_MODE, 4); % time based
write4ByteTxRx(port_num, PROTOCOL_VERSION, ID_BASE, 108, 2000); % acc
write4ByteTxRx(port_num, PROTOCOL_VERSION, ID_BASE, 112, 200); % vel

% Disable Dynamixel Torque
write1ByteTxRx(port_num, PROTOCOL_VERSION, ID_BASE, ADDR_PRO_TORQUE_ENABLE, TORQUE_DISABLE);
write1ByteTxRx(port_num, PROTOCOL_VERSION, ID_1, ADDR_PRO_TORQUE_ENABLE, TORQUE_DISABLE);

%enable torque
write1ByteTxRx(port_num, PROTOCOL_VERSION, ID_BASE, ADDR_PRO_TORQUE_ENABLE, TORQUE_ENABLE);
write1ByteTxRx(port_num, PROTOCOL_VERSION, ID_1, ADDR_PRO_TORQUE_ENABLE, TORQUE_ENABLE);
write1ByteTxRx(port_num, PROTOCOL_VERSION, ID_2, ADDR_PRO_TORQUE_ENABLE, TORQUE_ENABLE);
write1ByteTxRx(port_num, PROTOCOL_VERSION, ID_3, ADDR_PRO_TORQUE_ENABLE, TORQUE_ENABLE);
write1ByteTxRx(port_num, PROTOCOL_VERSION, ID_CLAW, ADDR_PRO_TORQUE_ENABLE, TORQUE_ENABLE);


%%
num_steps = 50;
time = linspace(0, 10*pi, num_steps);
sine_wave = sin(time);
cos_wave = cos(time);

amplitude = 500; 
offset = 2000;
sine_wave_array = round(amplitude * sine_wave) + offset;
cosine_wave_array = round(amplitude * cos_wave) + offset;
%%

i = 0; 
while (i<num_steps)
    i = i+1;

    write4ByteTxRx(port_num, PROTOCOL_VERSION, ID_BASE, ADDR_PRO_GOAL_POSITION, sine_wave_array(i));
    % write4ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID2, ADDR_PRO_GOAL_POSITION, cosine_wave_array(i));


    % dxl1_present_position = read4ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID1, ADDR_PRO_PRESENT_POSITION);
    % fprintf('[ID:%03d] Position: %03d\n', DXL_ID1, typecast(uint32(dxl1_present_position), 'int32'));
    % 
    % dxl_comm_result = getLastTxRxResult(port_num, PROTOCOL_VERSION);
    % dxl_error = getLastRxPacketError(port_num, PROTOCOL_VERSION);
    % 
    % if dxl_comm_result ~= COMM_SUCCESS
    %     fprintf('%s\n', getTxRxResult(PROTOCOL_VERSION, dxl_comm_result));
    % elseif dxl_error ~= 0
    %     fprintf('%s\n', getRxPacketError(PROTOCOL_VERSION, dxl_error));
    % end
    % 
    % if ~(abs(dxl_goal_position(index) - typecast(uint32(dxl1_present_position), 'int32')) > DXL_MOVING_STATUS_THRESHOLD)
    %     break;
    % end

    pause(0.1);
end

% Disable torque
write1ByteTxRx(port_num, PROTOCOL_VERSION, ID_BASE, ADDR_PRO_TORQUE_ENABLE, TORQUE_DISABLE);
write1ByteTxRx(port_num, PROTOCOL_VERSION, ID_1, ADDR_PRO_TORQUE_ENABLE, TORQUE_DISABLE);
write1ByteTxRx(port_num, PROTOCOL_VERSION, ID_2, ADDR_PRO_TORQUE_ENABLE, TORQUE_DISABLE);
write1ByteTxRx(port_num, PROTOCOL_VERSION, ID_3, ADDR_PRO_TORQUE_ENABLE, TORQUE_DISABLE);
write1ByteTxRx(port_num, PROTOCOL_VERSION, ID_CLAW, ADDR_PRO_TORQUE_ENABLE, TORQUE_DISABLE);


% Close port
closePort(port_num);
fprintf('Port Closed \n');

% Unload Library
unloadlibrary(lib_name);

close all;
clear all;

function Debug()
    % Debugger
    % dxl_comm_result = getLastTxRxResult(port_num, PROTOCOL_VERSION);
    % dxl_error = getLastRxPacketError(port_num, PROTOCOL_VERSION);
    % 
    % if dxl_comm_result ~= COMM_SUCCESS
    %     fprintf('%s\n', getTxRxResult(PROTOCOL_VERSION, dxl_comm_result));
    % elseif dxl_error ~= 0
    %     fprintf('%s\n', getRxPacketError(PROTOCOL_VERSION, dxl_error));
    % end
    % 
    % dxl1_present_position = read4ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID1, ADDR_PRO_PRESENT_POSITION);
    % fprintf('[ID:%03d] Position: %03d\n', DXL_ID1, typecast(uint32(dxl_present_position), 'int32'));
    
    % if ~(abs(dxl_goal_position(index) - typecast(uint32(dxl_present_position), 'int32')) > DXL_MOVING_STATUS_THRESHOLD)
    % end
end
