function [support_traj, P0, P1] = kinematic_inversion(legs, step, theta_a, leg_index, N_points)
    
    prism_start = 2; % starting point of the prismatic joint (vertical degree of freedom)
    M = [1 1 1 0 0 0];

    direction = [sind(theta_a) cosd(theta_a) 0]';
    q_stable = [prism_start pi/2 pi/2]; % stable point initialization (to be calibrated) 
    stable_point = legs(leg_index).fkine(q_stable);
    P = stable_point.t;
    P0 = +step/2*direction + P;    
    P1 = -step/2*direction + P;
    Ps = SE3(P0);
    Pe = SE3(P1);
    tj_points = ctraj(Ps, Pe, N_points);
    support_traj_sim = legs(leg_index).ikine(tj_points, 'mask', M, 'q0', q_stable, 'tol', 0.2);
    
    % Select only the joint variables of interest
%     plot_leg(legs(leg_index), support_traj_sim, P0, P1, 0, leg_index); % Plot results
    support_traj_sim = support_traj_sim(:, 2:3);
    
    
    % Trajectory saturation
    support_traj_sim = saturate_traj(support_traj_sim, 'rad', leg_index);
    
    % If dealing with left legs the positive rotation of the motor b is
    % the same as in simulation, always traslate in degree
    support_traj = rad2deg(support_traj_sim);
%     if (leg_index==4 || leg_index==5 || leg_index==6) 
%         support_traj = rad2deg(pi-support_traj_sim);
%     else
%         support_traj(:, 1) = rad2deg(pi-support_traj_sim(:, 1));
%         support_traj(:, 2) = rad2deg(support_traj_sim(:, 2));
%     end
end