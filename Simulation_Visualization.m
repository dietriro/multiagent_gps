

%% Properties
START_POINT = 1;            % Specify the number of the startpoint used for all robots
SAMPLE      = 5;            % Specify the number of sample of the startpoint used


%% Initialize
RANGE = 10;         % Sensor max range
N = 10;             % Number of Robots
S = 8;              % Number of Sensors
T = 1000;           % Number of Timesteps

robot_poses = zeros(N, T, 3);
sensor_robots = zeros(N, T, S);
sensor_targets = zeros(N, T, S);


%% Read input from CSV
% Column Description:
%
% 1, 2,   3,       4,       5,          6,           7,                            8-15,                    16-23,
% x, y, yaw, lin_vel, rot_vel, left_force, right_force, US[8] (starting at 0 going CCW),POIsensors[8] (same deal),R_POI,R_X,R_U

minX = 1000; 
minY = 1000;
maxX = -1000;
maxY = -1000;

for n = 1:N
    % Read input file for all robots
    input = csvread(['logfiles/env' int2str(START_POINT) '-' int2str(SAMPLE) '/roverlog' int2str(n-1) '.csv']);

    % Save robot poses and sensor measurements (0/1)
    robot_poses(n, :, :) = input(:, 1:3);
    robot_poses(n, :, 3) = -robot_poses(n, :, 3)+pi/2;
    sensor_robots(n, :, :) = fliplr(input(:, 8:15)) < RANGE;
    sensor_targets(n, :, :) = fliplr(input(:, 16:23)) > 0;

    % Save max and min values for adjust axe properties
    minX = min(min(input(:, 1)), minX);
    maxX = max(max(input(:, 1)), maxX);
    minY = min(min(input(:, 1)), minY);
    maxY = max(max(input(:, 1)), maxY);
end

minX = minX - 2;
maxX = maxX + 2;
minY = minY - 2;
maxY = maxY + 2;

%% Simulation

% Set figure position and size
figure('units','normalized','position',[.1 .06 .8 .83])

for t = 1:T
    for n = 1:N
        RobotVisualization(robot_poses(n, t, :), sensor_robots(n, t, :), sensor_targets(n, t, :)); 
    end
    
    xlim([minX maxX]);
    ylim([minY maxY]);
    grid on
    
    pause(0.0001);
   
    % Clear figure
    clf
end










