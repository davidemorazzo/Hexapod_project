function [] = move_rotation(legs, serial_obj, rotation_angle, N_points)

tj_support = zeros(N_points, 2, 6); 
tj_return = zeros(N_points, 2, 6);
tj_positioning = zeros(N_points/2, 2, 6);
tj_stabilize = zeros(N_points/2, 2, 6);
P0 = zeros(6, 3, 1); % initial point of the trajectory
P1 = zeros(6, 3, 1); % end point of the trajectory
group1 = [1 3 5]; % leg group 1
group2 = [2 4 6]; % leg group 2
[step1, theta_a1] = turn_generate_traj(legs, group1, rotation_angle);
    [step2, theta_a2] = turn_generate_traj(legs, group2, rotation_angle);
    j=1;
    k=2;
    for i=1:3
        [tj_support(:, :, j), P0(j, :, :), P1(j, :, :)] = kinematic_inversion(legs, step1(i), theta_a1(i), j, N_points);
        [tj_support(:, :, k), P0(k, :, :), P1(k, :, :)] = kinematic_inversion(legs, step2(i), theta_a2(i), k, N_points);
        j=j+2;
        k=k+2;
    end
    for i=1:6
        tj_positioning(:, :, i) = create_joint_traj(tj_support(:, :, i), N_points, 'positioning', i);
        tj_return(:, :, i) = create_joint_traj(tj_support(:, :, i), N_points, 'return', i);
        tj_stabilize(:, :, i) = create_joint_traj(tj_support(:, :, i), N_points, 'stabilizing', i);
%                 pause
    end

    execute_trajectory(serial_obj, tj_positioning(:, :, group1), [], ...
        'positioning', 'none', N_points);
    execute_trajectory(serial_obj, tj_support(:, :, group1), tj_positioning(:, :, group2), ...
        'execution', 'positioning', N_points);
    for i=1:3
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