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
    writePosition(servomotors(1), normalize_angle(tj_1(i,1), 'deg'));
    writePosition(servomotors(2), normalize_angle(tj_1(i,2), 'deg'));
    % leg 2
    writePosition(servomotors(3), normalize_angle(tj_2(i,1), 'deg'));
    writePosition(servomotors(4), normalize_angle(tj_2(i,2), 'deg'));
    % leg 3
    writePosition(servomotors(5), normalize_angle(tj_3(i,1), 'deg'));
    writePosition(servomotors(6), normalize_angle(tj_3(i,2), 'deg'));        
    % leg 4
    writePosition(servomotors(7), normalize_angle(tj_4(i,1), 'deg'));
    writePosition(servomotors(8), normalize_angle(tj_4(i,2), 'deg'));
    % leg 5
    writePosition(servomotors(9),  normalize_angle(tj_5(i,1), 'deg'));
    writePosition(servomotors(10), normalize_angle(tj_5(i,2), 'deg'));
    % leg 6
    writePosition(servomotors(11), normalize_angle(tj_6(i,1), 'deg'));
    writePosition(servomotors(12), normalize_angle(tj_6(i,2), 'deg'));
end

end