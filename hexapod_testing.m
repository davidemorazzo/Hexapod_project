clc
clear
close all
load angle.mat
%% Connection and setup
a = arduino('com14', 'uno',...
    'Libraries', {'Servo' 'Ultrasonic' 'I2C'},...
    'BaudRate', 128000);
s2 = servo(a, 'D10');
s1 = servo(a, 'D11');
%% Stable position (degrees)
angle1_n = normalize_angle(angle1, 'deg');
angle2_n = normalize_angle(angle2, 'deg');
writePosition(s1, angle1_n);
writePosition(s2, angle2_n);
%% Robot definition (all lengths are in cm)
l0 = 0;
l1 = 3.75;
% % l1 = 1;
% % l2 = 1.5;
% l2 = 6.05;
% offset_1 = -3.1;
% offset_2 = -2.25;
% % Create joints
% base = Prismatic('a', l0,'alpha', 0, 'qlim', [0 20]);
% leg1 = Revolute('a', l1, 'alpha', -pi/2, 'd', offset_1, 'qlim', [0 pi/2], 'offset', -pi/2);
% leg2 = Revolute('a', l2, 'qlim', [0 pi]);
% base_xyz = SE3();
% base_xyz.t = [0 0 -(offset_1-l2)-3];
% tool = SE3([0 offset_2 0]');
% % Create robot object
% leg = SerialLink([base, leg1, leg2], 'name', 'hexapod', 'base', base_xyz, 'tool', tool);
% % leg.plot([3 0 0])

leg = createLeg(5);
%% Trajectory planning

theta_a = 0; % motion direction [deg]
direction = [sind(theta_a) cosd(theta_a) 0]'; % direction vector of hexapod
D = 5; % step length
prism_start = 3;
circle_limits = false;

% determination of the stable point
% q_stable = [3, 0, pi/2];
q_stable = [prism_start deg2rad(180-angle11) deg2rad(180-angle10)];
stable_point = leg.fkine(q_stable);
P = stable_point.t;

% maximum and minimum circumferences to limit the leg range
r_min = 1.2*l1 - l0;
r_max = 3*l1 - l0;
j2_pos = [l0; 0; 0];

% starting point and end point
P0 = D/2 * direction + P;    
P1 = -D/2 * direction + P;
Ps = P0;
Pe = P1;
% theta = linspace(0, 2*pi);
% figure, hold on
% leg.plot([2 0 pi/2])
% plot3(j2_pos(1)+r_min*cos(theta), j2_pos(2)+r_min*sin(theta), 0*theta, 'k--')
% plot3(j2_pos(1)+r_max*cos(theta), j2_pos(2)+r_max*sin(theta), 0*theta, 'k--')
% plot3([P0(1); P1(1)], [P0(2); P1(2)], [0; 0], 'k--')
% plot3(P0(1),P0(2),P0(3), '*')
% plot3(P1(1),P1(2),P1(3), '*')
% text(P0(1), P0(2), P0(3), 'P0')
% text(P1(1), P1(2), P1(3), 'P1')
% axis([-20 20 -20 20 0 20])
% return

if(circle_limits==true)
% Detect if Ps and Pe are outside of the min and max circle
% if so then intersect the direction vector theta_a with the circle to find
% the Ps and Pe
% > Find points G and H
syms r c
line_eqn = P + direction * r;
line_eqn(3) = 0;
max_circle_eqn = j2_pos + r_max * [cos(c); sin(c); 0];
min_circle_eqn = j2_pos + r_min * [cos(c); sin(c); 0];
% Find H point
[r_sol, ~] = solve(line_eqn - min_circle_eqn, [r, c]);
H = double(subs(line_eqn, r, max(double(r_sol))));
% return
if ~isreal(H)
    % no intersection with small circle -> intersect with big circle
    [r_sol, ~] = solve(line_eqn - max_circle_eqn, [r, c]);
    H = double(subs(line_eqn, r, max(double(r_sol))));
end
% Find G point
[r_sol, ~] = solve(line_eqn - min_circle_eqn, [r, c]);
G = double(subs(line_eqn, r, min(double(r_sol))));
% return
if ~isreal(G)
    % no intersection with small circle -> intersect with big circle
    [r_sol, ~] = solve(line_eqn - max_circle_eqn, [r, c]);
    G = double(subs(line_eqn, r, min(double(r_sol))));
end
% > Assign Ps and Pe
% Ps -> starting point
% Pe -> ending point
PH = norm(P-H);
PG = norm(P-G);
if min([PH, PG]) >= D/2
    Ps = P0; Pe = P1;
elseif PH >= D/2 && PG <D/2
    Ps = G; Pe = P1;
elseif PH <D/2 && PG >= D/2
    Ps = P0; Pe = H;
elseif max([PG, PG]) < D/2
    Ps = G; Pe = H;
end
step_length = norm(Pe-Ps)

theta = linspace(0, 2*pi);
figure, hold on
leg.plot(q_stable)
plot3(j2_pos(1)+r_min*cos(theta), j2_pos(2)+r_min*sin(theta), 0*theta, 'k--')
plot3(j2_pos(1)+r_max*cos(theta), j2_pos(2)+r_max*sin(theta), 0*theta, 'k--')
plot3([Ps(1); Pe(1)], [Ps(2); Pe(2)], [0; 0], 'k--')
plot3(Ps(1),Ps(2),Ps(3), '*')
plot3(Pe(1),Pe(2),Pe(3), '*')
text(Ps(1), Ps(2), Ps(3)+1, 'Ps')
text(Pe(1), Pe(2), Pe(3)+1, 'Pe')
end

% return
% Conversion to SE3 object for ikine()
Ps = SE3(Ps);
Pe = SE3(Pe);
% Ps.t(3) = 13;
% Pe.t(3) = 13;
% tranimate(Ps, Pe)
% inverse kinematic trajectory
N_points = 20;                      % number of points in the trajectory
M = [1, 1, 1, 0, 0, 0];             % mask solve ikine only for x y z
operational_traj = ctraj(Ps, Pe, N_points);       % linear trajectory
joints_traj = ikine(leg, operational_traj, 'mask', M, 'q0', q_stable); % inverse kinematic
% joints_traj = leg.ikine(operational_traj, 'mask', M);
% return
%% Return trajectory

% From Pe to Ps lifting joint2 to max_heigth
max_height = pi/8; % [rad] return angle of joint 2
return_tj = jtraj(joints_traj(end,:), joints_traj(1,:), N_points);
return_tj(1:N_points/2,3) = linspace(joints_traj(end,3), max_height, N_points/2);
return_tj(N_points/2+1:end,3) = linspace(max_height, joints_traj(1,3), N_points/2);
% return
%% Commands to robot's joints
robot_direct_traj = normalize_angle(rad2deg(pi-joints_traj(:,2:3)), 'deg');
robot_return_traj = normalize_angle(rad2deg(pi-return_tj(:,2:3)), 'deg');
% return
while(1)
for i=1:N_points
    writePosition(s1,robot_direct_traj(i,1));
    writePosition(s2, robot_direct_traj(i,2));
end
for i=1:N_points
    writePosition(s1,robot_return_traj(i,1));
    writePosition(s2, robot_return_traj(i,2));
end
end
return
%% Plotting
% Figure representing the hexapod base height during the movement
figure
sgtitle('Support phase')
subplot(311)
plot(joints_traj(:,1))          % plot bot height during movement
ylabel('Height')
grid on
subplot(312)
plot(joints_traj(:,2))          % plot bot joint 1 during movement
xlabel('Trajectory point')
ylabel('\theta_1')
grid on
subplot(313)
plot(joints_traj(:,3))          % plot bot joint 2 during movement
xlabel('Trajectory point')
ylabel('\theta_2')
grid on

% add points and lines to the robot plot
figure, hold
axis([-20 20 -20 20 -2 20])
theta = linspace(0, 2*pi);
plot3(j2_pos(1)+r_min*cos(theta), j2_pos(2)+r_min*sin(theta), 0*theta, 'k--')
plot3(j2_pos(1)+r_max*cos(theta), j2_pos(2)+r_max*sin(theta), 0*theta, 'k--')
plot3(stable_point.t(1), stable_point.t(2), 0, 'k*')     % plot the stable point
plot3(Ps.t(1), Ps.t(2), Ps.t(3), 'k*')
plot3(Pe.t(1), Pe.t(2), Pe.t(3), 'k*')
plot3([Ps.t(1);Pe.t(1)], [Ps.t(2);Pe.t(2)], [Ps.t(3);Pe.t(3)], 'k--')
% plot3(H(1), H(2), H(3), 'ko');
% plot3(G(1), G(2), G(3), 'ko');

% Points labels
text(Ps.t(1), Ps.t(2)+0.3, Ps.t(3)+0.3, 'Ps')
text(Pe.t(1), Pe.t(2)+0.3, Pe.t(3)+0.3, 'Pe')
% text(H(1), H(2), H(3)+0.2, 'H');
% text(G(1), G(2), G(3)+0.2, 'G');

% Animate trajectory
leg.plot([joints_traj; return_tj],...
    'workspace', [-20 20 -20 20 -2 20], ...
    'noname', 'trail', 'b--.', 'loop')