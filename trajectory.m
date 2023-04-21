% Define constants
L = 0.5; % distance between wheels (m)
r = 0.1; % radius of wheels (m)

% Define initial conditions
x0 = 0; % initial x position (m)
y0 = 0; % initial y position (m)
theta0 = 0; % initial orientation (rad)

% Define reference trajectory
t = 0:0.1:10;
x_ref = sin(t);
y_ref = cos(t);

% Define control gains
k_x = 1; % proportional gain for x position
k_y = 1; % proportional gain for y position
k_theta = 1; % proportional gain for orientation

% Initialize state variables
x = x0;
y = y0;
theta = theta0;

% Initialize simulation variables
x_sim = zeros(size(t));
y_sim = zeros(size(t));
theta_sim = zeros(size(t));

% Simulate robot motion
for i = 1:length(t)
    % Compute control inputs
    x_error = x_ref(i) - x;
    y_error = y_ref(i) - y;
    theta_error = atan2(y_error, x_error) - theta;
    v = k_x * x_error;
    w = k_y * y_error + k_theta * theta_error;
    
    % Compute wheel speeds
    vr = v + L/2 * w;
    vl = v - L/2 * w;
    
    % Update state variables
    x_dot = v * cos(theta);
    y_dot = v * sin(theta);
    theta_dot = w;
    x = x + x_dot * 0.1;
    y = y + y_dot * 0.1;
    theta = theta + theta_dot * 0.1;
    
    % Store simulation results
    x_sim(i) = x;
    y_sim(i) = y;
    theta_sim(i) = theta;
end

% Plot results
figure;
plot(x_sim, y_sim);
title('Robot Trajectory');
xlabel('x (m)');
ylabel('y (m)');
