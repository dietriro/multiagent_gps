function y = RobotVisualization(init_pose, sensor_robots, sensor_targets)

% Robot position
px = init_pose(1);
py = init_pose(2);
% Robot orientation
pt = init_pose(3);

r = 2;                % Radius of circle
a = pi/4;               % Angle size of circle piece

x = sin(pt)*r+px;
y = cos(pt)*r+py;

X = [px, x];
Y = [py, y];

% Calculate all points of the circle
angle = pt;
for i = 1:8
    angle = angle + a;
    
    X = [X, sin(angle)*r+px];
    Y = [Y, cos(angle)*r+py];
end


% Print robot triangles
TRI = [ones(8,1), (2:9)', (3:10)'];
triplot(TRI, X, Y)



hold on

% Temporary sensor values
% sensor_robots = [0, 0, 0, 0, 1, 0, 1, 0];
% sensor_targets = [1, 0, 0, 0, 1, 0, 0, 1];


% Fill the triangles that sensed something with the 
% corresponding color
for i = 1:8
    if sensor_robots(i) && sensor_targets(i)
        fill([X(1), X(i+1:i+2)], [Y(1), Y(i+1:i+2)], 'yellow')
        hold on
    elseif sensor_robots(i)
        fill([X(1), X(i+1:i+2)], [Y(1), Y(i+1:i+2)], 'red')
        hold on
    elseif sensor_targets(i)
        fill([X(1), X(i+1:i+2)], [Y(1), Y(i+1:i+2)], 'green')
        hold on
    end
end

% Plot orientation of Robot
plot(X(1:2),Y(1:2),'black','LineWidth',2)

hold on





end