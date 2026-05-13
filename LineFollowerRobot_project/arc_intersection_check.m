function [intersect, px, py] = arc_intersection_check(p_sA, p_sB, arc)
%UNTITLED4 Summary of this function goes here
%   Detailed explanation goes here
% Line created from sensor array: y = m1 * x + c1
if (p_sA(1,1) == p_sB(1,1))
    m1 = 10e6;
else
    m1 = (p_sB(2) - p_sA(2)) / (p_sB(1) - p_sA(1));
end
c1 = p_sA(2) - m1 * p_sA(1);

% Arc created from curved trajectory path
center_x = arc(1);
center_y = arc(2);
% fprintf("Check arc center: cx = %8.3f, cy = %8.3f\n", center_x, center_y);
radius = arc(3);
theta_start = arc(4);
theta_end = arc(5);
theta = linspace(theta_start, theta_end, 20);
arc_x = center_x + radius * cos(theta);
arc_y = center_y + radius * sin(theta);

% We have quadratic function

syms x y
if p_sA(1,1) == p_sB(1,1)
    quad_eqn = (p_sA(1) - center_x) ^ 2 + (y - center_y) ^ 2 == radius ^ 2;
    S = solve(quad_eqn, y, real=true);
else 
    quad_eqn = (x - center_x) ^ 2 + (m1 * x + c1 - center_y) ^ 2 == radius ^ 2;
    S = solve(quad_eqn, x, real=true);
end
[solution_num, ~] = size(S);
if (solution_num == 0) 
    intersect = 0;
    px = 10e4;
    py = 10e4;
else
    if p_sA(1) == p_sB(1)
        px1 = p_sA(1); py1 = double(S(1));
        px2 = p_sA(1); py2 = double(S(2));
        p1_is_in_sensor_array = (py1 >= min(p_sA(2), p_sB(2))) && (py1 <= max(p_sA(2), p_sB(2)));
        p2_is_in_sensor_array = (py2 >= min(p_sA(2), p_sB(2))) && (py2 <= max(p_sA(2), p_sB(2)));
    else
        px1 = double(S(1)); py1 = m1 * px1 + c1;
        px2 = double(S(2)); py2 = m1 * px2 + c1;    
        p1_is_in_sensor_array = (px1 >= min(p_sA(1), p_sB(1))) && (px1 <= max(p_sA(1), p_sB(1)));
        p2_is_in_sensor_array = (px2 >= min(p_sA(1), p_sB(1))) && (px2 <= max(p_sA(1), p_sB(1)));
    end
    
    ptheta1 = mod(atan2(py1-center_y, px1-center_x), 2*pi);
    ptheta2 = mod(atan2(py2-center_y, px2-center_x), 2*pi);

    p1_is_in_arc = (ptheta1 <= theta_end) && (ptheta1 >= theta_start);
    p2_is_in_arc = (ptheta2 <= theta_end) && (ptheta2 >= theta_start);
       
    if (p1_is_in_sensor_array && p1_is_in_arc)
        intersect = 1;
        px = px1; py = py1;
        fprintf("Arc intersect - px: %8.2f\t py: %8.2f\t xc: %8.2f\t yc: %8.2f\t r: %8.2f\n", px1, py1, center_x, center_y, radius);
    elseif (p2_is_in_sensor_array && p2_is_in_arc)
        intersect = 1;
        px = px2; py = py2;
        fprintf("Arc intersect - px: %8.2f\t py: %8.2f\t xc: %8.2f\t yc: %8.2f\t r: %8.2f\n", px2, py2, center_x, center_y, radius);
    else
        intersect = 0;
        px = 10e4;
        py = 10e4;
    end
end
end
