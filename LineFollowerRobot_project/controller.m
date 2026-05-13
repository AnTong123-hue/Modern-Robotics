function [v, omg] = controller(err,kp,ki,kd, xr, yr, phi, T_sc)
%UNTITLED Summary of this function goes here
%   Detailed explanation goes here
yC_err = T_ce(2, 3);
yC_err_dot = 0;
yC_err_itg = 0;
kp = 5;
ki = 0;
kd = 0;
vd = 300;
yC_dot = kp * yC_err + ki * yC_err_itg + kd * yC_err_dot
xC_dot = sqrt(vd^2 - yC_dot^2)
p_cC_dot = [xC_dot; yC_dot; 1];
p_sC_dot = T_sc * p_cC_dot;
% Input control 
x_r = 175;
y_r = 0;
J = 1/x_r * [x_r * cos(phi) - y_r * sin(phi) x_r * sin(phi) + y_r * cos(phi); -sin(phi) cos(phi)];
control_input = J * p_sC_dot(1:2);
disp(control_input(1));
end