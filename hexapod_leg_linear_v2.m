clc
clear all
close all

%% Create robot
% Link length
l0 = 5;
l1 = 3.75;
l2 = 6.05;
o_1 = -3.1;
o_2 = -2.25;

% Create joints
base = Prismatic('qlim', 4, 'a', l0);
leg1 = Revolute('a',l1,'alpha', -pi/2, 'd',o_1);
leg2 = Revolute('a',l2,'alpha', pi, 'd',0);
% Set base frame
base_xyz = SE3();
base_xyz.t = [0 0 7.1500];
% Create robot object
leg = SerialLink([base, leg1, leg2],...
    'name', 'hexapod', ...
    'base', base_xyz, ...
    'tool', SE3([0 1 0]'));


%% Trajectory planning
v_x = 0;                        % [m/s] vel
v_y = 1;                        % [m/s] vel
v = [v_x;v_y;0];
support_time = 3;               % [s] time of support phase

delta = v * support_time;
direction = delta/norm(delta);  % step direction
D = norm(delta);           % step length   

% determination of the stable point
q_stable = [1, 0, pi/2];        % stable configuration
stable_point = leg.fkine(q_stable);
P = stable_point.t;

% starting point and end point
P0 = - D/2 * direction + P;    
P1 = + D/2 * direction + P;
Ps = SE3(P0);
Pe = SE3(P1);

% Inverse kinematic trajectory
N_points = 10;
M = [1 1 1 0 0 0];
tj_points = ctraj(Ps, Pe, N_points);
tic
joint_points = leg.ikine(tj_points, 'mask', M, 'q0', q_stable);
toc

%% Return trajectory

% From Pe to Ps lifting joint2 to max_heigth
max_heigth = 0.4; % [rad] return angle of joint 2
return_tj = jtraj(joint_points(end,:), joint_points(1,:), N_points);
return_tj(1:N_points/2,3) = linspace(joint_points(end,3), max_heigth, N_points/2);
return_tj(N_points/2+1:end,3) = linspace(max_heigth, joint_points(1,3), N_points/2);

%% Plotting
% Figure representing the hexapod base heigth during the movement
figure
sgtitle('Support phase')
subplot(311)
plot(joint_points(:,1))          % plot bot heigth during movement
ylabel('Heigth')
grid on
subplot(312)
plot(joint_points(:,2))          % plot bot heigth during movement
xlabel('Trajectory point')
ylabel('\theta_2')
grid on
subplot(313)
plot(joint_points(:,3))          % plot bot heigth during movement
xlabel('Trajectory point')
ylabel('\theta_3')
grid on

% add points and lines to the robot plot
figure, hold
plot3(stable_point.t(1), stable_point.t(2), 0, 'k*')     % plot the stable point
plot3(Ps.t(1), Ps.t(2), Ps.t(3), 'ko')
plot3(Pe.t(1), Pe.t(2), Pe.t(3), 'k*')
plot3([Ps.t(1);Pe.t(1)], [Ps.t(2);Pe.t(2)], [Ps.t(3);Pe.t(3)], 'k--')

% Animate trajectory
leg.plot([joint_points; return_tj],...
    'floorlevel', 0, ...
    'noname', 'trail', 'b--.', 'loop')

