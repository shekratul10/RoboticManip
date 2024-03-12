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
T_CLAW_CLOSE_CUBE = [1, CLAW_CUBE_CLOSE, 0, 0, 0];
DEFAULT_POS = [0,0.274,0,0.2048,0];

REST_POS = [2100, 1000, 2900, 2400];

C1_POS = [0.075, -0.20];
C2_POS = [0.225, 0];
C3_POS = [0.15, 0.15];
C4_POS = [0.125, -0.125];
C5_POS = [0.10, 0];
C6_POS = [0, 0.10];

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
% for i=1:length(IDS)-1
%     write4ByteTxRx(port_num, PROTOCOL_VERSION, IDS(i), ADDR_PRO_DRIVE_MODE, 4); % time based
%     write4ByteTxRx(port_num, PROTOCOL_VERSION, IDS(i), 108, 80); % acc
%     write4ByteTxRx(port_num, PROTOCOL_VERSION, IDS(i), 112, 80); % vel
% end

% Disable Dynamixel Torque
write1ByteTxRx(port_num, PROTOCOL_VERSION, ID_BASE, ADDR_PRO_TORQUE_ENABLE, TORQUE_DISABLE);
write1ByteTxRx(port_num, PROTOCOL_VERSION, ID_1, ADDR_PRO_TORQUE_ENABLE, TORQUE_DISABLE);
write1ByteTxRx(port_num, PROTOCOL_VERSION, ID_2, ADDR_PRO_TORQUE_ENABLE, TORQUE_DISABLE);
write1ByteTxRx(port_num, PROTOCOL_VERSION, ID_3, ADDR_PRO_TORQUE_ENABLE, TORQUE_DISABLE);
write1ByteTxRx(port_num, PROTOCOL_VERSION, ID_CLAW, ADDR_PRO_TORQUE_ENABLE, TORQUE_DISABLE);

% Enable Dynamixel Torque
write1ByteTxRx(port_num, PROTOCOL_VERSION, ID_BASE, ADDR_PRO_TORQUE_ENABLE, TORQUE_ENABLE);
write1ByteTxRx(port_num, PROTOCOL_VERSION, ID_1, ADDR_PRO_TORQUE_ENABLE, TORQUE_ENABLE);
write1ByteTxRx(port_num, PROTOCOL_VERSION, ID_2, ADDR_PRO_TORQUE_ENABLE, TORQUE_ENABLE);
write1ByteTxRx(port_num, PROTOCOL_VERSION, ID_3, ADDR_PRO_TORQUE_ENABLE, TORQUE_ENABLE);
write1ByteTxRx(port_num, PROTOCOL_VERSION, ID_CLAW, ADDR_PRO_TORQUE_ENABLE, TORQUE_ENABLE);

%% Initialise Robot to Default Position With Claw Open
[x, y, z, gamma] = read_curr_pos(port_num);
init_pos = [0, x, y, z, gamma];
task_list = [];
task_list = [init_pos; T_CLAW_OPEN; DEFAULT_POS];
% randomint = 0;
% while randomint==0
%     test = read4ByteTxRx(port_num, PROTOCOL_VERSION, ID_BASE, ADDR_PRO_PRESENT_POSITION);
%     disp(test);
%     [x, y, z, gamma] = read_curr_pos(port_num);
%     disp([x, y, z, gamma]);
%     pause(0.5);
% end

%% ---------- Task 2A Translation Waypoints ---------- %%
% 1 to 4
% task_list = [task_list;
% [0, C1_POS, 0.08, -pi/2]; % above
% [0, C1_POS, 0.06, -pi/2]; % cube pos 1
% T_CLAW_CLOSE_CUBE;
% [0, C1_POS, 0.08, -pi/2]; % above
% [0, C4_POS, 0.10, -pi/2]; % above
% [0, C4_POS, 0.045, -pi/2]; % cube pos 4
% T_CLAW_OPEN;
% ];
% 
% % 2 to 5
% task_list = [task_list;
% [0, C4_POS, 0.10, -pi/2]; % above
% [0, 0.2, 0, 0.1, -pi/2]; % midpoint
% [0, C2_POS, 0.06, -pi/2]; % cube pos 2
% T_CLAW_CLOSE_CUBE;
% [0, 0.2, 0, 0.1, -pi/2]; % midpoint
% [0, C5_POS, 0.10, -pi/2]; % above
% [0, C5_POS(1)-0.005, C5_POS(2), 0.045, -pi/2]; % cube pos 5
% T_CLAW_OPEN;
% ];
% 
% % 3 to 6
% task_list = [task_list;
% [0, C5_POS, 0.10, -pi/2]; % above
% [0, C3_POS, 0.08, -pi/2]; % above
% [0, C3_POS, 0.06, -pi/2]; % cube pos 3
% T_CLAW_CLOSE_CUBE;
% [0, C3_POS, 0.08, -pi/2]; % above
% [0, C6_POS, 0.10, -pi/2]; % above
% [0, C6_POS, 0.045, -pi/2]; % cube pos 6
% T_CLAW_OPEN;
% ];
% 
% task_list = [task_list; [0, C6_POS, 0.10, -pi/2]; DEFAULT_POS];


%% ---------- Task 2B Rotation Waypoints ---------- %%
% % flip 2
% task_list = [task_list;
% % rotation once
% [0, (C2_POS(1)+0.008), C2_POS(2), 0.052, -pi/2];
% T_CLAW_CLOSE_CUBE;
% [0, 0.2, 0, 0.1, -pi/2]; %mid point for rotation
% [0, 0.2, 0, 0.15, -pi/4];
% [0, 0.2, 0, 0.15, 0];
% [0, (C2_POS(1)-0.01), C2_POS(2), 0.05, 0]; % placement
% T_CLAW_OPEN;
% % default pos
% DEFAULT_POS
% % rotation twice
% [0, (C2_POS(1)+0.008), C2_POS(2), 0.052, -pi/2];
% T_CLAW_CLOSE_CUBE;
% [0, 0.2, 0, 0.1, -pi/2]; %mid point for rotation
% [0, 0.2, 0, 0.15, -pi/4];
% [0, 0.2, 0, 0.15, 0];
% [0, (C2_POS(1)-0.01), C2_POS(2), 0.05, 0]; % placement
% T_CLAW_OPEN;
% ];
% 
% flip 3
% task_list = [task_list;
% [0, 0.2, 0, 0.15, 0]; % raise to midpoint
% DEFAULT_POS;
% [0, C3_POS, 0.08, -pi/2]; % above
% [0, C3_POS+0.01, 0.052, -pi/2]; % at cube
% T_CLAW_CLOSE_CUBE;
% [0, C3_POS, 0.08, -pi/2]; % above
% [0, C3_POS, 0.15, 0]; % above + rotate
% [0, (C3_POS(1)-0.01), (C3_POS(2)-0.01), 0.05, 0]; % at cube
% T_CLAW_OPEN;
% ];
% 
% %flip 1
% task_list = [task_list;
% [0, C3_POS, 0.15, 0]; % raise
% [0, C1_POS, 0.15, 0];
% [0, C1_POS(1)-0.025, C2_POS(2)+0.075, 0.08, -pi/2]; % above
% [0, C1_POS, 0.08, -pi/2]; % above
% [0, C1_POS(1)+0.01, C1_POS(2)-0.01, 0.055, -pi/2]; % at cube
% T_CLAW_CLOSE_CUBE;
% [0, C1_POS, 0.08, -pi/2]; % above
% [0, C1_POS, 0.15, 0]; % above + rotate
% [0, C1_POS(1)-0.01, C1_POS(2)+0.01, 0.05, 0]; % at cube
% T_CLAW_OPEN;
% ];


%% ---------- Task 2C Stacking Waypoints ---------- %%
% stacking all on five
% rotate 2
task_list = [task_list;
% rotation once
[0, (C2_POS(1)+0.008), C2_POS(2), 0.052, -pi/2];
T_CLAW_CLOSE_CUBE;
[0, 0.2, 0, 0.1, -pi/2]; %mid point for rotation
[0, 0.2, 0, 0.15, -pi/4];
[0, 0.2, 0, 0.15, 0];
[0, (C2_POS(1)-0.01), C2_POS(2), 0.05, 0]; % placement
T_CLAW_OPEN;
% default pos
DEFAULT_POS
% rotation twice
[0, (C2_POS(1)+0.008), C2_POS(2), 0.052, -pi/2];
T_CLAW_CLOSE_CUBE;
[0, 0.2, 0, 0.1, -pi/2]; %mid point for rotation
[0, 0.2, 0, 0.15, -pi/4];
[0, 0.2, 0, 0.15, 0];
[0, (C2_POS(1)-0.01), C2_POS(2), 0.05, 0]; % placement
T_CLAW_OPEN;
];

task_list = [task_list;
    [0, 0.2, 0, 0.15, 0];
    DEFAULT_POS;
 ];

% translating cube 2 -> 5
task_list = [task_list;
[0, C2_POS, 0.06, -pi/2]; % cube pos 2
T_CLAW_CLOSE_CUBE;
[0, C5_POS, 0.10, -pi/2]; % above
[0, C5_POS, 0.052, -pi/2]; % cube pos 5
T_CLAW_OPEN;
];

%rotate 3
task_list = [task_list;
[0, 0.2, 0, 0.15, 0]; % raise to midpoint
DEFAULT_POS;
[0, C3_POS, 0.08, -pi/2]; % above
[0, C3_POS+0.01, 0.052, -pi/2]; % at cube
T_CLAW_CLOSE_CUBE;
[0, C3_POS, 0.08, -pi/2]; % above
[0, C3_POS, 0.15, 0]; % above + rotate
[0, (C3_POS(1)-0.01), (C3_POS(2)-0.01), 0.05, 0]; % at cube
T_CLAW_OPEN;
];
% translating cube 3 -> 5
task_list = [task_list;
[0, (C3_POS(1)-0.01), (C3_POS(2)-0.01), 0.10, 0]; % above
[0, (C3_POS(1)-0.05), (C3_POS(2)-0.05), 0.10, -pi/2]; % midpoint
[0, C3_POS, 0.08, -pi/2]; % above
[0, C3_POS, 0.055, -pi/2]; % cube pos 3
T_CLAW_CLOSE_CUBE;
[0, C3_POS, 0.08, -pi/2]; % above
[0, C5_POS, 0.075, -pi/2]; % cube pos 5
T_CLAW_OPEN;
];

task_list = [task_list;
[0, C5_POS, 0.12, -pi/2]; % raise
[0, C1_POS(1)-0.025, C1_POS(2)+0.075, 0.08, -pi/2]; % above
[0, C1_POS, 0.08, -pi/2]; % above
[0, C1_POS(1)+0.01, C1_POS(2)-0.01, 0.055, -pi/2]; % at cube
T_CLAW_CLOSE_CUBE;
[0, C1_POS, 0.08, -pi/2]; % above
[0, C1_POS, 0.15, 0]; % above + rotate
[0, C1_POS(1)-0.01, C1_POS(2)+0.01, 0.05, 0]; % at cube
T_CLAW_OPEN;
[0, C1_POS * 2 / 3, 0.15, -pi/2];
];
% translating cube 1 -> 5
task_list = [task_list;
[0, C1_POS, 0.08, -pi/2]; % above
[0, C1_POS, 0.055, -pi/2]; % cube pos 1
T_CLAW_CLOSE_CUBE;
[0, C1_POS, 0.08, -pi/2]; % above
[0, C5_POS, 0.10, -pi/2]; % cube pos 5
T_CLAW_OPEN;
];

task_list = [task_list; [0, C5_POS, 0.10, -pi/2]; DEFAULT_POS];

%% ---------- EXECUTION ---------- %%
for i= 1:size(task_list, 1)
    task_handler(task_list(i, :), port_num);
end

%% return to rest
for j=1:10
    for i=1:4
        write4ByteTxRx(port_num, PROTOCOL_VERSION, IDS(i), ADDR_PRO_GOAL_POSITION, REST_POS(i));
    end
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