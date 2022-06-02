function [step, theta_a] = turn(legs, group, rotation_angle)
q_stable = [2 pi/2 pi/2];
for i=1:3
    leg_index = group(i);
    stable_point(:, i) = legs(leg_index).fkine(q_stable).t;
    l_radius(i) = norm(stable_point(:, i)); % scalar length of circumference radius
    direction(:, i) = [-stable_point(2, i); stable_point(1, i); 0];
    direction(:, i) = direction(:, i)./norm(direction(:, i));
    theta_a(i) = 90-atan2d(direction(2, i), direction(1, i));
end
step = deg2rad(rotation_angle)*l_radius;

end