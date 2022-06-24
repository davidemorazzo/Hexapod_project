function [step, theta_a] = turn_generate_traj(legs, group, rotation_angle)

q_stable = [2 pi/2 pi/2];
stable_point = zeros(3, 3);
direction = zeros(3, 3);
l_radius = zeros(3, 1);
theta_a = zeros(3, 1);

for i=1:3
    leg_index = group(i); % extract leg index from group vector
    stable_point(:, i) = legs(leg_index).fkine(q_stable).t; % find stable point via forward kinematics
    l_radius(i) = norm(stable_point(:, i)); % scalar length of circumference radius
    direction(:, i) = [-stable_point(2, i); stable_point(1, i); 0]; % find normal vector to current direction
    direction(:, i) = direction(:, i)./norm(direction(:, i)); % unit vector of normal direction
    theta_a(i) = 90-atan2d(direction(2, i), direction(1, i));
end
step = deg2rad(rotation_angle)*l_radius;

end