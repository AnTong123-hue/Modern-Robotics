0 1%% Initialize map
% If trajectory is a line, it will be defined by two points' position
% If trajectory is an arc, it will be defined by center point, radius,
% start angle, end angle
tr0 = [0, 3000, 0, 500, 0, 0]
tr1 = [1, 500, 500, 500, pi/2, 3*pi/2]
tr2 = [0, 500, 1000, 1000-800*tand(22.5), 1000, 0]

% Split track 
tr3_1 = [1, 1000-800*tand(22.5), 1800, 800, 3*pi/2, 7*pi/4]
tr4_1 = [0, 1000+800*tand(22.5)*cosd(45), 1000+800*tand(22.5)*sind(45), 1500-800*tand(22.5)*cosd(45), 1500-800*tand(22.5)*sind(45), 0]
tr5_1 = [1, 1500+800*tand(22.5), 1500-800, 800, pi/2, 3*pi/4]
tr6_1 = [0, 1500+800*tand(22.5), 1500, 3000, 1500, 0]

tr3_2 = [1, 1000-800*tand(22.5), 1000-800, 800, pi/4, pi/2]
tr4_2 = [0, 1000+800*tand(22.5)*cosd(45), 1000-800*tand(22.5)*sind(45), 1500-800*tand(22.5)*cosd(45), 500+800*tand(22.5)*sind(45),0]
tr5_2 = [1, 1500+800*tand(22.5), 500+800, 800, 5*pi/4, 3*pi/2]
tr6_2 = [0, 1500+800*tand(22.5), 500, 3000, 500, 0]

target_tr_1 = [tr0; tr1; tr2; tr3_1; tr4_1; tr5_1; tr6_1];
target_tr_2 = [tr0; tr1; tr2; tr3_2; tr4_2; tr5_2; tr6_2];

% Create operation virtual 2D plane
env = figure('Name', 'Environment');

ax  = axes('Parent', env);
axis equal
title(ax, "Car-Like Robot Simulation Environment");

% Draw targeted trajectory 1 & 2
trajectory_plot(target_tr_1, env, ax);
trajectory_plot(target_tr_2, env, ax);

%% Run simulation again with control error recording
phi0=pi; T0 = [cos(phi0) -sin(phi0) 3000; sin(phi0) cos(phi0) 0; 0 0 1];
sample_freq = 10; % 1000kHz
dt = 1 / sample_freq;
t_start = 0;
t_end   = 1000;
vd = 500;
v = 1000;
omg = 0;
qdot = zeros(1,3);
phi = pi;
x = 800;
y = 0;
T_sb = [cos(phi) -sin(phi) x; sin(phi) cos(phi) y; 0 0 1];
l = 175;
figure(env)
axis(ax)
cla(ax)
hold on

% Draw targeted trajectory 1 & 2
trajectory_plot(target_tr_1, env, ax);
trajectory_plot(target_tr_2, env, ax);

% PID parameters
kp = 0.1;
ki = 0.1;
kd = 0;
yC_err_itg = 0;
yC_err_prev = 0;
err_hist = [];
nearest_path = -1;
for t = t_start:dt:t_end
    pause(dt);
    % Forward Kinematics
    G = [0 1; cos(phi) 0; sin(phi) 0];
    qdot = subs(G, phi, 0) * [vd; omg];
    phi = phi + qdot(1) * dt;
    x = x + qdot(2) * dt;
    y = y + qdot(3) * dt;
    x_ref = x + l * cos(phi);
    y_ref = y + l * sin(phi);
    T_sb = [cos(phi) -sin(phi) x; sin(phi) cos(phi) y; 0 0 1];
    
    % Mobile Robot's base & frame
    plot(x, y, 'bo', 'MarkerSize', 5, 'MarkerFaceColor', 'r'); 
    plot([x, x_ref], [y, y_ref], 'g', LineWidth=1.5);
  
    % Dexterous sensor arrangement
    varphi = atan(l * omg / v); 
    T_bc = [cos(varphi) -sin(varphi) l; sin(varphi) cos(varphi) 0; 0 0 1];
    T_sc = T_sb * T_bc;
    p_cA = [0; 30; 1]; p_cB = [0;-30; 1];
    p_sA = double(T_sc * p_cA);
    p_sB = double(T_sc * p_cB);
    plot([p_sA(1), p_sB(1)], [p_sA(2), p_sB(2)], 'r-');
    
    % Control error check 
    [T_ce, intersect, nearest_path] = intersection_check(T_sc, target_tr_1, nearest_path);
    T_se = T_sc * T_ce;
    if intersect == true
       plot(T_se(1,3), T_se(2,3), 'y+')
    end

    % Feedback control 
    yC_err = T_ce(2, 3);
    err_hist = [err_hist yC_err];
    yC_err_dot = (yC_err - yC_err_prev) / dt;
    yC_err_prev = yC_err;
    yC_err_itg = yC_err_itg + yC_err * dt;
    
    % Input control 
    omg = kp * yC_err + ki * yC_err_itg + kd * yC_err_dot;
    % Error log
    fprintf('Time %10.3f: phi = %10.3f\t x = %10.3f\t y = %10.3f\t px_err = %10.3f \t py_err = %10.3f\n', t, phi, x, y, T_ce(1, 3), T_ce(2, 3));
end