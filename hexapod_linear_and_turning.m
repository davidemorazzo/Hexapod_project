% Simulation of 3 hexapod legs trajectory specifying linear and rotational
% velocities
clc
close all
clear all
format compact

%% Robot object creation
% Robot dimensions
N_legs = 3;
% Link lengths of each leg
l0 = 1;
l1 = 1;
l2 = 2;
% Leg base coordinates with respect of center of mass (columns)
bases{1} = SE3([2 3 1]');
bases{2} = SE3([2 -3 1]');
bases{3} = SE3().Rz(pi);
bases{3}.t = [-2 0 1]';

% Legs creation
% NOTE: leg 1 and 2 on the rigth side, leg 3 on the left side
for i=1:3
    base = Prismatic('qlim', 2, 'a', l0);
    leg1 = Revolute('a',l1,'alpha', -pi/2, 'd',0);
    leg2 = Revolute('a',l2,'alpha', pi, 'd',0);
    % Create robot object
    hexapod_leg = SerialLink([base, leg1, leg2],...
        'name', sprintf('leg_%s',i), ...
        'base', bases{i});
%     Vector with all the legs
    legs{i} = hexapod_leg;
end

%% Tragectory planning
% Reference velocities
gait_time = 0.4;  % [s] support phase duration
v = 0;          % [-] linear velocity
omega_z = 0.9;  % [-] angular velocity

% determination of the stable point for each leg
q_stable = [1, 0, pi/2];
for i=1:N_legs
    stable_point(:,i) = legs{i}.fkine(q_stable).t;
end

% center of mass radius, 
% if is =inf the motion is only linear
% if is =0 motion is only rotational
COM_traj_radius = - v/omega_z;
radius_center = [COM_traj_radius;0;0];
l1_radius = radius_center - stable_point(:,1);
l2_radius = radius_center - stable_point(:,2);
l3_radius = radius_center - stable_point(:,3);
radiuses = [l1_radius l2_radius l3_radius];

% determination of each leg direction normalized
d1 = [-l1_radius(2); l1_radius(1); 0];
d2 = [-l2_radius(2); l2_radius(1); 0];
d3 = [-l3_radius(2); l3_radius(1); 0];
d1 = d1./norm(d1);
d2 = d2./norm(d2);
d3 = d3./norm(d3);
directions = [d1 d2 d3];
directions(isnan(directions)) = 1*sign(v);

% Determination of each leg step length
COM_arc_len = v * gait_time;
arc_angle = COM_arc_len / COM_traj_radius;

% starting point and end point
for i=1:3
    if omega_z == 0
        D = v*gait_time;
    else
        alpha_omega = omega_z * gait_time;
        D = alpha_omega * norm(radiuses(:,i)) + v*gait_time;        
    end
    P0(:,i) = - D/2 * directions(:,i) + stable_point(:,i);
    P1(:,i) = + D/2 * directions(:,i) + stable_point(:,i);
end


%% Plotting
figure, hold, grid
axis([-5 5 -5 5 0 6])
theta = linspace(0,2*pi)';
% SE3(radius_center).plot()
SE3([0;0;1]).plot()
plot3(radius_center(1)+sin(theta)*norm(l1_radius), ...
    radius_center(2)+cos(theta)*norm(l1_radius), ...
    theta*0, 'r--')
plot3(radius_center(1)+sin(theta)*norm(l3_radius), ...
    radius_center(2)+cos(theta)*norm(l3_radius), ...
    theta*0, 'r--')

% Plot starting and ending points
plot3(P0(1,:), P0(2,:), P0(3,:), 'k+');
plot3(P1(1,:), P1(2,:), P1(3,:), 'ko');
% Plotting the trajectory lines
plot3([P0(1,1); P1(1,1)],[P0(2,1); P1(2,1)],[P0(3,1); P1(3,1)],'k')
plot3([P0(1,2); P1(1,2)],[P0(2,2); P1(2,2)],[P0(3,2); P1(3,2)],'k')
plot3([P0(1,3); P1(1,3)],[P0(2,3); P1(2,3)],[P0(3,3); P1(3,3)],'k')

% Draw square for the body
p1 = [2,4,1];
p2 = [2,-4,1];
p3 = [-2,-4,1];
p4 = [-2,4,1];
p = [p1;p2;p3;p4;p1];
line(p(:,1), p(:,2), p(:,3))

% % plot the legs
% legs{1}.plot(q_stable,...
%     'workspace', [-5 5 -5 5 0 6], ...
%     'floorlevel', 0, ...
%     'noname', 'trail', 'b--.')
% legs{2}.plot(q_stable,...
%     'noname', 'trail', 'b--.')
% legs{3}.plot(q_stable,...
%     'noname', 'trail', 'b--.')



