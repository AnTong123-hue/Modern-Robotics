function [T_ce, intersect, nearest_path] = intersection_check(T_sc, target_trajectory, prev_nearest_path)
% Summary of this function goes here
% Check intercections of sensor arrangement with all segments of trajectory 
T_ce = eye(3, 3);
T_ce(1,3) = 10e4;
T_ce(2,3) = 10e4;
p_cA = [0; 30; 1]; p_cB = [0; -30; 1];
p_sA = double(T_sc * p_cA); p_sB = double(T_sc * p_cB);
%disp(round(p_sA, 2));
%disp(round(p_sB, 2));
fprintf("Check intersection: pxA = %8.3f, pxB = %8.3f\n", p_sA(1), p_sB(1));
[tr_num, ~] = size(target_trajectory);
intersect = 0;

% At first check again with previous neareast path
continue_check = 1;
if prev_nearest_path > -1
    tr_info = target_trajectory(prev_nearest_path, :);
    if (tr_info(1) == 0) 
        [intersect, px, py] = line_intersection_check(p_sA, p_sB, tr_info(2:5));
    else
        [intersect, px, py] = arc_intersection_check(p_sA, p_sB, tr_info(2:6));
    end
    if (intersect == true)
        T_ce(:, 3) = T_sc\[px; py; 1];
        nearest_path = prev_nearest_path;
        continue_check = 0;
    else 
        continue_check = 1;
    end 
end
% Check with all other path if neareast path failed
if continue_check
    for i=1:1:tr_num
        tr_info = target_trajectory(i, :);
        if (tr_info(1) == 0) 
            [intersect, px, py] = line_intersection_check(p_sA, p_sB, tr_info(2:5));
        else
            [intersect, px, py] = arc_intersection_check(p_sA, p_sB, tr_info(2:6));
        end
        if (intersect == true)
            T_ce(:, 3) = T_sc\[px; py; 1];
            nearest_path = i;
            % fprintf("Check intersection: px = %8.3f, py = %8.3f\n", px, py);
            break;
        end 
    end
end

end