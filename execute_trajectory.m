function execute_trajectory(servomotors, f_traj, r_traj, leg_group, n_points)
% Execution of the specified triangle gait phase trajectories. One group
% forward trajectory and the other group return trajectory

if leg_group == 1
    tj_1 = f_traj(1,:,:);
    tj_2 = r_traj(2,:,:);
    tj_3 = f_traj(3,:,:);
    tj_4 = r_traj(4,:,:);
    tj_5 = f_traj(5,:,:);
    tj_6 = r_traj(6,:,:);
elseif leg_group == 2
    tj_1 = r_traj(1,:,:);
    tj_2 = f_traj(2,:,:);
    tj_3 = r_traj(3,:,:);
    tj_4 = f_traj(4,:,:);
    tj_5 = r_traj(5,:,:);
    tj_6 = f_traj(6,:,:);
end


for i=1:n_points
    % leg 1
    writePosition(servomotors(1), tj_1(1, i, 1));
    writePosition(servomotors(2), tj_1(1, i, 2));
    % leg 2
    writePosition(servomotors(3), tj_2(1, i,1));
    writePosition(servomotors(4), tj_2(1, i,2));
    % leg 3
    writePosition(servomotors(5), tj_3(1, i, 1));
    writePosition(servomotors(6), tj_3(1, i, 2));        
    % leg 4
    writePosition(servomotors(7), tj_4(1, i, 1));
    writePosition(servomotors(8), tj_4(1, i, 2));
    % leg 5
    writePosition(servomotors(9),  tj_5(1, i, 1));
    writePosition(servomotors(10), tj_5(1, i, 2));
    % leg 6
    writePosition(servomotors(11), tj_6(1, i, 1));
    writePosition(servomotors(12), tj_6(1, i, 2));
end

end