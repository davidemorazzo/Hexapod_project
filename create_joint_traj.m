function [joints_traj] = create_joint_traj(support_traj, N_points, traj_type)

max_height = 23; % max height of motor b position, imposed to move in the air
N_rounded = floor(N_points/4); % rounding to have an integer number of points
q_stable = [90 90]'; % stable position of the joints, to configure

switch traj_type
    case 'return'
        joints_traj(1:N_points/2, :) = jtraj(support_traj(end,:), [support_traj(N_points/2, 1) max_height], N_points/2);
        joints_traj(N_points/2+1:N_points, :) = jtraj([support_traj(N_points/2+1, 1) max_height], support_traj(1, :), N_points/2);
    case 'positioning'
        % Positioning trajectory
        joints_traj(1:N_rounded, :) = jtraj(q_stable, [support_traj(N_rounded, 1) max_height], N_rounded);
        joints_traj(N_rounded+1:N_points/2, :) = jtraj([support_traj(N_rounded, 1) max_height], support_traj(1, :), ...
            N_points/2-N_rounded);
    case 'stabilize'
        joints_traj(1:N_rounded, :) = jtraj(support_traj(end, :), [support_traj(N_rounded, 1) max_height], N_rounded);
        joints_traj(N_rounded+1:N_points/2, :) = jtraj([support_traj(N_rounded, 1) max_height], q_stable, ...
            N_points/2-N_rounded);
    otherwise
        joints_traj = [];
end
joints_traj = saturate_traj(joints_traj, 'deg');
end