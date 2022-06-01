function [joints_traj] = create_joint_traj(support_traj, N_points, traj_type, leg_index)

max_height_right = 180-23; % max height of motor b position, imposed to move in the air
max_height_left = 23;
N_rounded = floor(N_points/4); % rounding to have an integer number of points
q_stable = [90 90]'; % stable position of the joints, to configure

if(leg_index==4 || leg_index==5 || leg_index==6)
    switch traj_type
        case 'return'
            joints_traj(1:N_points/2, :) = jtraj(support_traj(end,:), [support_traj(N_points/2, 1) max_height_right], N_points/2);
            joints_traj(N_points/2+1:N_points, :) = jtraj([support_traj(N_points/2+1, 1) max_height_right], support_traj(1, :), N_points/2);
        case 'positioning'
            % Positioning trajectory
            joints_traj(1:N_rounded, :) = jtraj(q_stable, [support_traj(N_rounded, 1) max_height_right], N_rounded);
            joints_traj(N_rounded+1:N_points/2, :) = jtraj([support_traj(N_rounded, 1) max_height_right], support_traj(1, :), ...
                N_points/2-N_rounded);
        case 'stabilize'
            joints_traj(1:N_rounded, :) = jtraj(support_traj(end, :), [support_traj(N_rounded, 1) max_height_right], N_rounded);
            joints_traj(N_rounded+1:N_points/2, :) = jtraj([support_traj(N_rounded, 1) max_height_right], q_stable, ...
                N_points/2-N_rounded);
        otherwise
            joints_traj = [];
    end
else
    switch traj_type
        case 'return'
            joints_traj(1:N_points/2, :) = jtraj(support_traj(end,:), [support_traj(N_points/2, 1) max_height_left], N_points/2);
            joints_traj(N_points/2+1:N_points, :) = jtraj([support_traj(N_points/2+1, 1) max_height_left], support_traj(1, :), N_points/2);
        case 'positioning'
            % Positioning trajectory
            joints_traj(1:N_rounded, :) = jtraj(q_stable, [support_traj(N_rounded, 1) max_height_left], N_rounded);
            joints_traj(N_rounded+1:N_points/2, :) = jtraj([support_traj(N_rounded, 1) max_height_left], support_traj(1, :), ...
                N_points/2-N_rounded);
        case 'stabilize'
            joints_traj(1:N_rounded, :) = jtraj(support_traj(end, :), [support_traj(N_rounded, 1) max_height_left], N_rounded);
            joints_traj(N_rounded+1:N_points/2, :) = jtraj([support_traj(N_rounded, 1) max_height_left], q_stable, ...
                N_points/2-N_rounded);
        otherwise
            joints_traj = [];
    end
end
% joints_traj = saturate_traj(joints_traj, 'deg', leg_index);
% figure
% plot( 1:length(joints_traj(:, 1)), joints_traj(:, 1));
% str = sprintf("%s leg %d motor A", traj_type, leg_index);
% title(str)
% figure
% plot( 1:length(joints_traj(:, 2)), joints_traj(:, 2));
% str = sprintf("%s leg %d motor B", traj_type, leg_index);
% title(str)
end