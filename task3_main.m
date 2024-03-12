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

IDS = [ID_BASE, ID_1, ID_2, ID_3, ID_CLAW];

BAUDRATE                    = 115200;
DEVICENAME                  = 'COM5';
                                            
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

%% ----- SET CONSTANTS ----------- %%

CLAW_OPEN = 1300;
CLAW_CUBE_CLOSE = 2430;
CLAW_PEN_CLOSE = 2360;

T_CLAW_OPEN = [1, CLAW_OPEN, 0, 0, 0];
T_CLAW_CLOSE_PEN = [1, CLAW_PEN_CLOSE, 0, 0, 0];
DEFAULT_POS = [0,0.274,0,0.2048,0];

REST_POS = [2100, 1000, 2900, 2400];

PICKUP_POS = [0, -0.15];
OFFSET_PEN = 0.08;

BL = [0.2,0.06];
BR = [0.2,0.14];
TR = [0.125,0.14];

% ADDR_MAX_POS = 48;
% ADDR_MIN_POS = 52;
% MAX_POS = 3400;
% MIN_POS = 600;

% % Set max position limit
% write4ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID1, ADDR_MAX_POS, MAX_POS);
% write4ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID2, ADDR_MAX_POS, MAX_POS);
% % Set min position limit
% write4ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID1, ADDR_MIN_POS, MIN_POS);
% write4ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID2, ADDR_MIN_POS, MIN_POS);

%% ---------------------------------- %%

% Change Drive Mode and Profile
for i=1:length(IDS)
    write4ByteTxRx(port_num, PROTOCOL_VERSION, IDS(i), ADDR_PRO_DRIVE_MODE, 4); % time based
    write4ByteTxRx(port_num, PROTOCOL_VERSION, IDS(i), 108, 80); % acc
    write4ByteTxRx(port_num, PROTOCOL_VERSION, IDS(i), 112, 80); % vel
end

% Enable Dynamixel Torque
for i=1:length(IDS)
    write1ByteTxRx(port_num, PROTOCOL_VERSION, IDS(i), ADDR_PRO_TORQUE_ENABLE, TORQUE_ENABLE);
end

%% Initialise Robot to Default Position With Claw Open
[x, y, z, gamma] = read_curr_pos(port_num);
init_pos = [0, x, y, z, gamma];
task_list = [init_pos; T_CLAW_OPEN; DEFAULT_POS];

%% ---------- Pick Up Pen ---------- %%
task_list = [task_list;
[0, PICKUP_POS, 0.1, -pi/2];
[0, PICKUP_POS, 0.02, -pi/2];
T_CLAW_CLOSE_PEN;
[0, PICKUP_POS, 0.1, -pi/2];
[0, PICKUP_POS, 0.15, 0];
[0, [BL, OFFSET_PEN+0.05], 0];
[0, [BL, OFFSET_PEN], 0]];

pause(0.5);

%% ---------- Draw Triangle ---------- %%
LINE_STEPS = 20;
task_list = [task_list;
            linear_interpolation([BL, OFFSET_PEN], [BR, OFFSET_PEN], LINE_STEPS);
            linear_interpolation([BR, OFFSET_PEN], [TR, OFFSET_PEN], LINE_STEPS*2);
            linear_interpolation([TR, OFFSET_PEN], [BL, OFFSET_PEN], LINE_STEPS)];

%% ---------- Draw Arc ---------- %%
ARC_STEPS = 100;
task_list = [task_list; arc_interpolation(ARC_STEPS, OFFSET_PEN)];

pause(0.5);
task_list = [task_list;
[0,BR,OFFSET_PEN+0.05,0];
[0, PICKUP_POS, 0.15, 0];
[0, PICKUP_POS, 0.1, -pi/2];
T_CLAW_OPEN;
];

%% ---------- EXECUTION ---------- %%
for i =1:size(task_list, 1)
    task_handler(task_list(i, :), port_num);
end

% return to rest pos
for j=1:10
    for i=1:4
        write4ByteTxRx(port_num, PROTOCOL_VERSION, IDS(i), ADDR_PRO_GOAL_POSITION, REST_POS(i));
    end
    pause(0.1);
end

% Disable Dynamixel Torque
for i=1:length(IDS)-1
    write1ByteTxRx(port_num, PROTOCOL_VERSION, IDS(i), ADDR_PRO_TORQUE_ENABLE, TORQUE_DISABLE);
end

% Close port
closePort(port_num);
fprintf('Port Closed \n');

% Unload Library
unloadlibrary(lib_name);

close all;
clear all;