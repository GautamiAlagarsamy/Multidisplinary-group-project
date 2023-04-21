% Define the trajectory path as a series of (x,y) positions
path = [0,0;
        0.0713,0.5203;
        0.1412,1.0324;
        0.2096,1.5463;
        0.2764,2.0620;
        0.3414,2.5795;
        0.4048,3.0986;
        0.4665,3.6193;
        0.5266,4.1416;
        0.5850,4.6655;
        0.6417,5.1910;
        0.6967,5.7180;
        0.7500,6.2465;
        0.8016,6.7766;
        0.8515,7.3082;
        0.8996,7.8412;
        0.9461,8.3758;
        0.9908,8.9118;
        1.0337,9.4493;
        1.0749,9.9881];

% Define the robot's kinematic constraints
wheel_radius = 0.1; % meters
robot_width = 0.5; % meters

% Define the initial robot pose
x0 = 0;
y0 = 0;
theta0 = 0;

% Calculate the robot's pose at each point along the path
x = x0;
y = y0;
theta = theta0;
poses = zeros(size(path,1), 3);
for i = 2:size(path,1)
    % Calculate the distance traveled by the left and right wheels
    dl = sqrt((path(i,1)-path(i-1,1))^2 + (path(i,2)-path(i-1,2))^2);
    dr = dl;
    
    % Calculate the robot's orientation
    alpha = atan2(path(i,2)-path(i-1,2), path(i,1)-path(i-1,1)) - theta;
    
    % Calculate the robot's pose
    x = x + (dl+dr)/2 * cos(theta+alpha);
    y = y + (dl+dr)/2 * sin(theta+alpha);
    theta = theta + (dl-dr) / robot_width;
    
    % Store the robot's pose
    poses(i,:) = [x, y, theta];
end

% Plot the scatter plot of the robot's position
scatter(path(:,1), path(:,2), 'b');
hold on;
scatter(poses(:,1), poses(:,2), 'r');
xlabel('X Position (m)');
ylabel('Y Position (m)');
legend('Desired Path', 'Actual Path');

% Plot the 3D scan of the robot's path
figure;
plot3(poses(:,1), poses(:,2), poses(:,3));
xlabel('X Position (m)');
ylabel('Y Position (m)');
zlabel('Orientation (rad)');
