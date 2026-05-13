function [intersect, px, py] = line_intersection_check(p_sA, p_sB, line)
% UNTITLED3 Summary of this function goes here
% Detailed explanation goes here
% Line created from sensor array: y = m1 * x + c1
if (p_sA(1) == p_sB(1))
    m1 = 10e6;
else
    m1 = (p_sB(2) - p_sA(2)) / (p_sB(1) - p_sA(1));
end
c1 = p_sA(2) - m1 * p_sA(1);

% Line created from trajectory's straight segment: y = m2 * x + c2
m2 = (line(4) - line(2)) / (line(3) - line(1));
c2 = line(2) - m2 * line(1);
% fprintf("Check line connected points: px1 = %8.3f, py1 = %8.3f, px2 = %8.3f, py2 = %8.3f\n", line(1), line(2), line(3), line(4));

% Intersection between two lines
if (m1 == m2)
    intersect = 0;
    px = 10e4;
    py = 10e4;
else
    px = (c2 - c1) / (m1 - m2);
    py = m1 * px + c1;
    if p_sA(1) == p_sB(1)
        is_in_sensor_array = (py >= min(p_sA(2), p_sB(2))) && (py <= max(p_sA(2), p_sB(1)));
    else
        is_in_sensor_array = (px >= min(p_sA(1), p_sB(1))) && (px <= max(p_sA(1), p_sB(1)));
    end

    is_in_path = (px >= min(line(1), line(3))) && (px <= max(line(1), line(3)));
    
    
    if (is_in_path && is_in_sensor_array)
        intersect = 1;
        fprintf("Line intersect - px: %8.2f\t x1: %8.2f\t x2: %8.2f\t valid: %d\n", px, line(1), line(3), is_in_path);
    else
        intersect = 0;
        px = 10e4;
        py = 10e4;
    end
end
end