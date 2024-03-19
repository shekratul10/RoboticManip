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
DEVICENAME                  = 'COM7';
                                            
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

CUBE_OFFSET = 0.055;
STACK_TWO = 0.08;
STACK_THREE = 0.10;

C1_POS = [0.175, -0.175];
C2_POS = [0.175, 0.10];
C3_POS = [0, 0.175];
C4_POS = [0, -0.15];
C5_POS = [0.10, 0.10];
C6_POS = [0.225, 0];

%% ---------------------------------- %%

% Change Drive Mode and Profile
for i=1:length(IDS)-1
    write4ByteTxRx(port_num, PROTOCOL_VERSION, IDS(i), ADDR_PRO_DRIVE_MODE, 4); % time based
    write4ByteTxRx(port_num, PROTOCOL_VERSION, IDS(i), 108, 80); % acc 80
    write4ByteTxRx(port_num, PROTOCOL_VERSION, IDS(i), 112, 80); % vel 80
end

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


%% ---------- Task 2A Translation Waypoints ---------- %%
% 1 to 4
% Note: horizontal clamp
task_list = [task_list;
[0, C1_POS, 0.08, 0]; % above
[0, C1_POS, CUBE_OFFSET+0.05, 0]; % cube pos 1
T_CLAW_CLOSE_CUBE;
[0, C1_POS, 0.08, 0]; % above
[0, C4_POS, 0.08, 0]; % above
[0, C4_POS, CUBE_OFFSET+0.05, 0]; % cube pos 4
T_CLAW_OPEN;
[0, C4_POS, 0.08, 0]; % above
DEFAULT_POS;
];

% 2 to 5
task_list = [task_list;
[0, C2_POS, 0.10, -pi/2]; % above
[0, C2_POS, CUBE_OFFSET, -pi/2]; % cube pos 2
T_CLAW_CLOSE_CUBE;
[0, C2_POS, 0.10, -pi/2]; % above
[0, C5_POS, 0.10, -pi/2]; % above
[0, C5_POS, CUBE_OFFSET, -pi/2]; % cube pos 5
T_CLAW_OPEN;
];

% 3 to 6
task_list = [task_list;
[0, C5_POS, 0.10, -pi/2]; % above
% Use flat hand
[0, C3_POS, 0.10, -pi/2]; % above
[0, C3_POS, CUBE_OFFSET, -pi/2]; % cube pos 3
T_CLAW_CLOSE_CUBE;
[0, C3_POS, 0.10, -pi/2]; % above
[0, C6_POS, 0.068, -pi/2]; % above
[0, C6_POS, CUBE_OFFSET, -pi/2]; % cube pos 6
T_CLAW_OPEN;
];

task_list = [task_list; [0, C6_POS, 0.068, -pi/2]; DEFAULT_POS];


%% ---------- Task 2B Rotation Waypoints ---------- %%
% Translate 1 to 4 and flip same time
task_list = [task_list;
[0, C1_POS, 0.08, 0]; % above
[0, C1_POS, CUBE_OFFSET+0.025, 0]; % cube pos 1
T_CLAW_CLOSE_CUBE;
[0, C1_POS, 0.08, 0]; % above
[0, C1_POS(1) * 0.9, C1_POS(2) * 0.9, 0.15, 0]; % midpoint
[0, 0.10, 0.10, 0.1, -pi/2]; % rotation
[0, C4_POS, 0.08, -pi/2]; % above
[0, C4_POS, CUBE_OFFSET, -pi/2]; % cube pos 4
T_CLAW_OPEN;
[0, C4_POS, 0.08, 0]; % above
];

% Translate 4 back to 1
task_list = [task_list;
[0, C4_POS, 0.08, 0]; % above
[0, C4_POS, CUBE_OFFSET+0.05, 0]; % cube pos 4
T_CLAW_CLOSE_CUBE;
[0, C4_POS, 0.08, 0]; % above
[0, C1_POS, 0.08, 0]; % above
[0, C1_POS, CUBE_OFFSET+0.05, 0]; % cube pos 1
T_CLAW_OPEN;
[0, C1_POS, 0.08, 0]; % above
];

%rotate 2
task_list = [task_list;
[0, C2_POS, 0.15, 0];
[0, C2_POS, 0.08, -pi/2]; % above
[0, C2_POS, CUBE_OFFSET, -pi/2]; % at cube
T_CLAW_CLOSE_CUBE;
[0, C2_POS, 0.08, -pi/2]; % above
[0, C2_POS, 0.15, 0]; % above + rotate
[0, C2_POS, CUBE_OFFSET, 0]; % at cube
T_CLAW_OPEN;
[0, C2_POS, 0.15, 0]; % above
];

%rotate 2 (second time)
task_list = [task_list;
[0, C2_POS, 0.15, 0];
[0, C2_POS, 0.08, -pi/2]; % above
[0, C2_POS, CUBE_OFFSET, -pi/2]; % at cube
T_CLAW_CLOSE_CUBE;
[0, C2_POS, 0.08, -pi/2]; % above
[0, C2_POS, 0.15, 0]; % above + rotate
[0, C2_POS, CUBE_OFFSET, 0]; % at cube
T_CLAW_OPEN;
[0, C2_POS, 0.15, 0]; % above
];

%rotate 3
task_list = [task_list;
[0, C3_POS, 0.12, -pi/2]; % above
[0, C3_POS, CUBE_OFFSET, -pi/2]; % at cube
T_CLAW_CLOSE_CUBE;
[0, C3_POS, 0.12, -pi/2]; % above
[0, C3_POS, 0.15, 0]; % above + rotate
[0, C3_POS, CUBE_OFFSET, 0]; % at cube
T_CLAW_OPEN;
[0, C3_POS, 0.10, 0]; % above
DEFAULT_POS;
];


%% ---------- Task 2C Stacking Waypoints ---------- %%
% Translate 1 to 4 and flip same time
task_list = [task_list;
[0, C1_POS, 0.08, 0]; % above
[0, C1_POS, CUBE_OFFSET+0.025, 0]; % cube pos 1
T_CLAW_CLOSE_CUBE;
[0, C1_POS, 0.08, 0]; % above
[0, C1_POS(1) * 0.9, C1_POS(2) * 0.9, 0.15, 0]; % midpoint
[0, 0.10, 0.10, 0.1, -pi/2]; % rotation
[0, C4_POS, 0.08, -pi/2]; % above
[0, C4_POS, CUBE_OFFSET, 0]; % cube pos 4
T_CLAW_OPEN;
[0, C4_POS, 0.08, 0]; % above
];

%rotate 2
task_list = [task_list;
[0, C2_POS, 0.15, 0];
[0, C2_POS, 0.08, -pi/2]; % above
[0, C2_POS, CUBE_OFFSET, -pi/2]; % at cube
T_CLAW_CLOSE_CUBE;
[0, C2_POS, 0.08, -pi/2]; % above
[0, C2_POS, 0.15, 0]; % above + rotate
[0, C2_POS, CUBE_OFFSET, 0]; % at cube
T_CLAW_OPEN;
[0, C2_POS, 0.15, 0]; % above
];

%rotate 2 (second time)
task_list = [task_list;
[0, C2_POS, 0.15, 0];
[0, C2_POS, 0.08, -pi/2]; % above
[0, C2_POS, CUBE_OFFSET, -pi/2]; % at cube
T_CLAW_CLOSE_CUBE;
[0, C2_POS, 0.08, -pi/2]; % above
[0, C2_POS, 0.15, 0]; % above + rotate
% Dont drop cube
];

% translate 2 to 4
task_list = [task_list;
[0, C4_POS, 0.08, 0]; % above
[0, C4_POS, STACK_TWO, 0]; % cube pos 4 2nd stack
T_CLAW_OPEN;
[0, C4_POS, STACK_TWO+0.05, 0]; % above
DEFAULT_POS;
];

%rotate 3
task_list = [task_list;
[0, C3_POS, 0.12, -pi/2]; % above
[0, C3_POS, CUBE_OFFSET, -pi/2]; % at cube
T_CLAW_CLOSE_CUBE;
[0, C3_POS, 0.12, -pi/2]; % above
[0, C3_POS, 0.15, 0]; % above + rotate
% dont drop cube
];

% translate 3 to 4
task_list = [task_list;
[0, C4_POS, STACK_THREE+0.05, 0]; % above
[0, C4_POS, STACK_THREE, 0]; % cube pos 4 3rd stack
T_CLAW_OPEN;
[0, C4_POS, STACK_TWO+0.05, 0]; % above
DEFAULT_POS;
];



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