% Load LIDAR data
lidar_data = load('lidar_data.mat');

% Define grid parameters
resolution = 0.1;   % meters per cell
grid_size = [100 100];   % number of cells in x and y directions
origin = [-5 -5];   % coordinates of the bottom-left corner of the grid

% Initialize binary occupancy grid
grid = robotics.BinaryOccupancyGrid(grid_size(1), grid_size(2), 1/resolution);
grid.GridLocationInWorld = origin;

% Set occupancy threshold
threshold = 0.5;

% Convert LIDAR data to polar coordinates
angles = linspace(-pi/2, pi/2, size(lidar_data,2));
ranges = lidar_data';

% Convert polar coordinates to Cartesian coordinates
x = ranges .* cos(angles);
y = ranges .* sin(angles);

% Convert Cartesian coordinates to grid coordinates
x_indices = round((x - origin(1)) / resolution);
y_indices = round((y - origin(2)) / resolution);

% Update grid with occupied cells
for i = 1:length(x_indices)
    if (x_indices(i) > 0) && (x_indices(i) <= grid_size(1)) && ...
       (y_indices(i) > 0) && (y_indices(i) <= grid_size(2))
        grid.setOccupancy([x_indices(i), y_indices(i)], 1);
    end
end

% Display the grid
figure;
show(grid);
title('Binary Occupancy Grid');

% Save the grid to a file
save('grid.mat', 'grid');
% Load grid from file
load('grid.mat');

% Display the grid
figure;
show(grid);
title('Binary Occupancy Grid');