function [success] = trajectory_plot(target_trajectory,fig, ax)
%TRAJECTORY_PLOT Summary of this function goes here
%   Detailed explanation goes here
figure(fig);
axes(ax); 
hold on;
[tr_num, nothing] = size(target_trajectory);
for i=1:1:tr_num
    tr_info = target_trajectory(i, :);
    if (tr_info(1) == 0) 
        % Plot a line with two points' position
        X = [tr_info(2), tr_info(4)];
        Y = [tr_info(3), tr_info(5)];
        plot(X, Y, 'Color', 'black', LineWidth=2);
    else
        % Plot an arc with center point, radius and angle information
        r = tr_info(4);
        center = [tr_info(2), tr_info(3)];
        start_angle = tr_info(5);
        end_angle = tr_info(6);
    
        % Generate angles from start to end
        theta = linspace(start_angle, end_angle, 100);
    
        % Position of points on targeted arc
        X = center(1) + r * cos(theta);
        Y = center(2) + r * sin(theta);
    
        % Plot the arc
        plot(X, Y, Color='black', LineWidth=2);
        plot(center(1), center(2), 'r+');
    end
    success = 1;
end
end

