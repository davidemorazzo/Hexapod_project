function OK = plot_leg(leg, traj, P0, P1, joint_info, leg_index)

l1 = 3.75;
r_min = 1.2*l1;
r_max = 3*l1;

if(joint_info==1)
figure
title_support_phase = sprintf("Support phase joint %d", leg_index);
sgtitle(title_support_phase)
subplot(311)
plot(traj(:,1))          % plot bot height during movement
ylabel('Height')
grid on
subplot(312)
plot(traj(:,2))          % plot bot joint 1 during movement
xlabel('Trajectory point')
ylabel('\theta_1')
grid on
subplot(313)
plot(traj(:,3))          % plot bot joint 2 during movement
xlabel('Trajectory point')
ylabel('\theta_2')
grid on
end

% add points and lines to the robot plot
figure, hold
title_plot = sprintf("Leg %d", leg_index);
title(title_plot);
axis([-20 20 -20 20 -2 20])
theta = linspace(0, 2*pi);
plot3(r_min*cos(theta), r_min*sin(theta), 0*theta, 'k--')
plot3(r_max*cos(theta), r_max*sin(theta), 0*theta, 'k--')

plot3(P0(1), P0(2), P0(3), 'k*')
plot3(P1(1), P1(2), P1(3), 'k*')
plot3([P0(1) P1(1)], [P0(2) P1(2)], [P0(3) P1(3)], 'k--')


% Points labels
% text(Ps.t(1), Ps.t(2)+0.3, Ps.t(3)+0.3, 'Ps')
% text(Pe.t(1), Pe.t(2)+0.3, Pe.t(3)+0.3, 'Pe')

% Animate trajectory
leg.plot(traj,...
    'workspace', [-20 20 -20 20 -2 20], ...
    'noname', 'trail', 'b--.')

OK = 1;
end