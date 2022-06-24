function [] = move_linear(legs, serial_obj, step, theta_a, N_routines, N_points)

tj_support = zeros(N_points, 2, 6); 
tj_return = zeros(N_points, 2, 6);
tj_positioning = zeros(N_points/2, 2, 6);
tj_stabilize = zeros(N_points/2, 2, 6);
P0 = zeros(6, 3, 1); % initial point of the trajectory
P1 = zeros(6, 3, 1); % end point of the trajectory
group1 = [1 3 5]; % leg group 1
group2 = [2 4 6]; % leg group 2

for i=1:6
    % inverse kinematics for each leg
    [tj_support(:, :, i), P0(i, :, :), P1(i, :, :)] = kinematic_inversion(legs, step, theta_a, i, N_points);
%                 P0(i, :, :), P1(i, :, :)
%                 plot3([P0(i, 1) P1(i, 1)], [P0(i, 2) P1(i, 2)], [P0(i, 3) P1(i, 3)], 'k--');
%                 hold on

    % create joints' routines for each leg
%                 plot_leg(legs(i), )
    tj_positioning(:, :, i) = create_joint_traj(tj_support(:, :, i), N_points, 'positioning', i);
    
    tj_return(:, :, i) = create_joint_traj(tj_support(:, :, i), N_points, 'return', i);
    tj_stabilize(:, :, i) = create_joint_traj(tj_support(:, :, i), N_points, 'stabilizing', i);
end
% Group 1 -> legs 1,3,5; Group 2 -> legs 2,4,6
execute_trajectory(serial_obj, tj_positioning(:, :, group1), [], ...
    'positioning', 'none', N_points);
execute_trajectory(serial_obj, tj_support(:, :, group1), tj_positioning(:, :, group2), ...
    'execution', 'positioning', N_points);
for i=1:N_routines
    execute_trajectory(serial_obj, tj_return(:, :, group1), tj_support(:, :, group2), ...
        'return', 'execution', N_points);
    execute_trajectory(serial_obj, tj_support(:, :, group1), tj_return(:, :, group2), ...
        'execution', 'return', N_points);
end
execute_trajectory(serial_obj, tj_stabilize(:, :, group1), tj_support(:, :, group2), ...
    'stabilizing', 'execution', N_points);
execute_trajectory(serial_obj, [], tj_stabilize(:, :, group2), ...
    'none', 'stabilizing', N_points);

end