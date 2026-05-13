% Mobile Robot Animation using hgtransform
clear; clc; close all;

%% 1. Setup the Fixed Line
t_path = linspace(0, 10, 200);
path_x = t_path;
path_y = sin(t_path);

figure('Name', 'Smooth Robot Animation', 'Color', 'w');
hold on; grid on; axis equal;
xlim([-1, 11]); ylim([-2, 2]);

% Draw the fixed line ONCE. We will never clear or redraw this.
plot(path_x, path_y, 'k--', 'LineWidth', 2); 

%% 2. Create the Robot Shape and Transform Group
% Create a transform object. This is an invisible container we will move.
robot_transform = hgtransform; 

% Define a simple robot shape (a triangle pointing forward)
% The shape is defined centered at (0,0) pointing along the X-axis
robot_length = 0.6;
robot_width = 0.4;
robot_x_coords = [robot_length/2, -robot_length/2, -robot_length/2];
robot_y_coords = [0, robot_width/2, -robot_width/2];

% Draw the robot as a filled polygon and 'parent' it to the transform group
patch('XData', robot_x_coords, 'YData', robot_y_coords, ...
      'FaceColor', 'b', 'Parent', robot_transform);

%% 3. Simulation Loop
dt = 0.1;
v = 0.5;
x = path_x(1); 
y = path_y(1);
theta = pi/4; % Start at an angle

for i = 1:200
    % --- KINEMATICS (Replace with your actual control logic) ---
    % For this animation demo, we will just make it follow the path exactly
    x = path_x(i);
    y = path_y(i);
    
    % Calculate heading (tangent to the path)
    if i < 200
        theta = atan2(path_y(i+1) - y, path_x(i+1) - x);
    end
    
    % --- ANIMATION UPDATE ---
    % Create a translation matrix (moves the object to x, y, z)
    T = makehgtform('translate', [x, y, 0]);
    
    % Create a rotation matrix (rotates the object around the Z axis by theta)
    R = makehgtform('zrotate', theta);
    
    % Combine them (Translation * Rotation) and apply to the transform group
    robot_transform.Matrix = T * R; 
    
    % Use drawnow to flush the graphics pipeline and render the frame
    % 'drawnow limitrate' is even better for smooth, fast animations
    drawnow limitrate; 
    
    pause(0.05); % Control visual speed
end