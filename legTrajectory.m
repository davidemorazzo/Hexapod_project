function [positioning_traj, direct_traj, return_traj] = legTrajectory(legs, step_length, theta_a, N_points, leg_index, visualize)
% returning the direct and return trajectory normalized (0 to 1) and in the robot angle

load angle.mat

% legs -> array of leg objects
% theta_a -> angle of robot motion, 0: moving forward 
% N_points -> number of points to use for inverse kinematics
% step_length -> length of the leg step in cm
% leg_index -> index of the leg to determine the inverse kinematics for

% determination of the stable point
prism_start = 2; % starting point of the prismatic joint (vertical degree of freedom)
M = [1 1 1 0 0 0];
saturation_a_max = 2/3*pi; % saturation limits
saturation_a_min = pi/3;
saturation_b_max = 3/4*pi;
saturation_b_min = pi/4;
max_height = pi/8; % [rad] return angle of joint 2

direction = [sind(theta_a) cosd(theta_a) 0]';
q_stable = [prism_start deg2rad(180-angles(leg_index).a) deg2rad(180-angles(leg_index).b)]; % stable configuration, trasformation 
% from simulation angle to robot angle is always -> sim_angle = 180-robot_angle
stable_point = legs(leg_index).fkine(q_stable);
P = stable_point.t;

% starting point and end point
P0 = +step_length/2*direction + P;    
P1 = -step_length/2*direction + P;

Ps = SE3(P0);
Pe = SE3(P1);

% Inverse kinematic trajectory
tj_points = ctraj(Ps, Pe, N_points);
direct_traj_sim = legs(leg_index).ikine(tj_points, 'mask', M, 'q0', q_stable, 'tol', 0.2);

% Positioning trajectory
N_rounded = floor(N_points/4); % rounding to have an integer number of points
positioning_traj_sim(1:N_rounded, :) = jtraj(q_stable, ...
    [direct_traj_sim(N_rounded, 1) direct_traj_sim(N_rounded, 2) max_height], N_rounded);
positioning_traj_sim(N_rounded+1:N_points/2, :) = jtraj([direct_traj_sim(N_rounded, 1) direct_traj_sim(N_rounded, 2) max_height], ...
    direct_traj_sim(end,:), N_points/2-N_rounded-1);


% Return trajectory
return_traj_sim(1:N_points/2, :) = jtraj(direct_traj_sim(end,:), ...
    [direct_traj_sim(N_points/2,1) direct_traj_sim(N_points/2,2) max_height], N_points/2);
return_traj_sim(N_points/2+1:N_points, :) = jtraj([direct_traj_sim(N_points/2,1) direct_traj_sim(N_points/2,2) max_height], ...
    direct_traj_sim(1, :), N_points/2);

% Saturation
direct_traj_sim(:, 2) = min(saturation_a_max, max(saturation_a_min, direct_traj_sim(:, 2))); % motor a 
direct_traj_sim(:, 3) = min(saturation_b_max, max(saturation_b_min, direct_traj_sim(:, 3))); % motor b
return_traj_sim(:, 2) = min(saturation_a_max, max(saturation_a_min, return_traj_sim(:, 2))); 
return_traj_sim(:, 3) = min(saturation_b_max, max(saturation_b_min, return_traj_sim(:, 3)));
positioning_traj_sim(:, 2) = min(saturation_a_max, max(saturation_a_min, positioning_traj_sim(:, 2))); 
positioning_traj_sim(:, 3) = min(saturation_b_max, max(saturation_b_min, positioning_traj_sim(:, 3)));

% Return the trajectories
if visualize==1
    plot_leg(legs(leg_index), direct_traj_sim, return_traj_sim, P0, P1, 0, leg_index); % Plot results
end

direct_traj_sim = direct_traj_sim(:, 2:3);
return_traj_sim = return_traj_sim(:, 2:3);
positioning_traj_sim = positioning_traj_sim(:, 2:3);

if (leg_index==4 || leg_index==5 || leg_index==6) 
    direct_traj = rad2deg(pi-direct_traj_sim);
    return_traj = rad2deg(pi-return_traj_sim);
    positioning_traj = rad2deg(pi-positioning_traj_sim);
else
    direct_traj(:, 1) = rad2deg(pi-direct_traj_sim(:, 1));
    direct_traj(:, 2) = rad2deg(direct_traj_sim(:, 2));
    return_traj(:, 1) = rad2deg(pi-return_traj_sim(:, 1));
    return_traj(:, 2) = rad2deg(return_traj_sim(:, 2));
    positioning_traj(:, 1) = rad2deg(pi-positioning_traj_sim(:, 1));
    positioning_traj(:, 2) = rad2deg(positioning_traj_sim(:, 2));
end
end