clear
close 
clc
format compact

% Lista degli stati
% - wait for input

% - steady          (stabilizzato)

% - walk_forward
%   - right_legs
%   - left_legs

% - walk_backward
%   - right_legs
%   - left_legs

% - rotate_left
%   - right_legs
%   - left_legs

% - rotate_right
%   - right_legs
%   - left_legs
%% Connection and setup

% load angle.mat
% com_port = 'COM15';
% serial_obj = serialport(com_port, 57600);
% serial_obj.configureTerminator("CR/LF")
% pause(1);
% arduino_servo_pos(serial_obj, 90*ones(6, 1), 1);
% arduino_servo_pos(serial_obj, 90*ones(6, 1), 2);

%% Robot's leg creation (simulation)
close all
legs = [createLeg(1) createLeg(2) createLeg(3) createLeg(4) createLeg(5) createLeg(6)]';
% figure
% hold on
% legs(1).plot([0 pi/2 pi/2])
% for i=2:6
%     legs(i).plot([0 pi/2 pi/2]);
% end
%% State machine
current_state = 'wait_for_input';
next_state = '';
N_points = 20;      % points in the leg trajectory
visualize = 0;  % visualization of simulation results
% Tridimensional matrices: first dimension -> leg index, second dimension -> Number of trajectory points, 
% third dimension -> motor index (anca e ginocchio)
% we divide the total trajectory of each leg in 4 parts, support part is
% the only one requiring inverse kinematics for each points, the other are:
% - positioning: prepare the leg to move to execute the support phase
% - return: return from the end of the support phase to the start
% - stabilize: return from the end of the support phase to the stable point
tj_support = zeros(N_points, 2, 6); 
tj_return = zeros(N_points, 2, 6);
tj_positioning = zeros(N_points/2, 2, 6);
tj_stabilize = zeros(N_points/2, 2, 6);
P0 = zeros(6, 3, 1);
P1 = zeros(6, 3, 1);
group1 = [1 3 5];
group2 = [2 4 6];

% State machine
while true
%     arduino_servo_pos(serial_obj, 90*ones(12, 1)); 
    switch current_state
        
        % ----- State wait_for_input -------
        case'wait_for_input'
            user_input = input("Insert command\n1 -> Walk forward" + ...
                "\n2 -> Walk backward" + ...
                "\n3 -> Rotate rigth" + ...
                "\n4 -> Rotate left" + ...
                "\n5 -> Steady\n");
            user_input = int8(user_input);
            switch user_input
                case 1
                    next_state = 'walk_forward';
                case 2
                    next_state = 'walk_backward';
                case 3
                    next_state = 'rotate_right';
                case 4
                    next_state = 'rotate_left';
                case 5
                    next_state = 'steady';
                otherwise
                    next_state = 'wait_for_input';
            end

        % ----- state walk_forward -----
        case 'walk_forward'
            step = 3; % step length
            theta_a = 0; % direction of the hexapod [deg] (0 -> forward, 90 -> right)
            % Creation of trajectories
            for i=1:6
                % inverse kinematics for each leg
                [tj_support(:, :, i), P0(i, :, :), P1(i, :, :)] = kinematic_inversion(legs, step, theta_a, i, N_points);
%                 P0(i, :, :), P1(i, :, :)
%                 plot3([P0(i, 1) P1(i, 1)], [P0(i, 2) P1(i, 2)], [P0(i, 3) P1(i, 3)], 'k--');
%                 hold on

                % create joints' routines for each leg
%                 tj_positioning_tmp = create_joint_traj(tj_support(i, :, :), N_points, 'positioning');
%                 plot_leg(legs(i), )
                tj_positioning(:, :, i) = create_joint_traj(tj_support(:, :, i), N_points, 'positioning', i);
                
                tj_return(:, :, i) = create_joint_traj(tj_support(:, :, i), N_points, 'return', i);
                tj_stabilize(:, :, i) = create_joint_traj(tj_support(:, :, i), N_points, 'stabilizing', i);
                pause
            end
            % Group 1 -> legs 1,3,5; Group 2 -> legs 2,4,6
            execute_trajectory(serial_obj, tj_positioning(:, :, group1), [], ...
                'positioning', 'none', N_points);
            execute_trajectory(serial_obj, tj_support(:, :, group1), tj_positioning(:, :, group2), ...
                'execution', 'positioning', N_points);
            for i=1:4
                execute_trajectory(serial_obj, tj_return(:, :, group1), tj_support(:, :, group2), ...
                    'return', 'execution', N_points);
                execute_trajectory(serial_obj, tj_support(:, :, group1), tj_return(:, :, group2), ...
                    'execution', 'return', N_points);
            end
            execute_trajectory(serial_obj, tj_stabilize(:, :, group1), tj_support(:, :, group2), ...
                'stabilizing', 'execution', N_points);
            execute_trajectory(serial_obj, [], tj_stabilize(:, :, group2), ...
                'none', 'stabilizing', N_points);
            % Next state evaluation
            next_state = 'wait_for_input';
        
        % ----- state walk_backward -------
        case 'walk_backward'
            step = 3; % step length
            theta_a = 180; % direction of the hexapod [deg] (0 -> forward, 90 -> right)
            % Creation of trajectories
            for i=1:6
                % inverse kinematics for each leg
                [tj_support(:, :, i), P0(i, :, :), P1(i, :, :)] = kinematic_inversion(legs, step, theta_a, i, N_points);
%                 P0(i, :, :), P1(i, :, :)
%                 plot3([P0(i, 1) P1(i, 1)], [P0(i, 2) P1(i, 2)], [P0(i, 3) P1(i, 3)], 'k--');
%                 hold on

                % create joints' routines for each leg
%                 tj_positioning_tmp = create_joint_traj(tj_support(i, :, :), N_points, 'positioning');
%                 plot_leg(legs(i), )
                tj_positioning(:, :, i) = create_joint_traj(tj_support(:, :, i), N_points, 'positioning', i);
                
                tj_return(:, :, i) = create_joint_traj(tj_support(:, :, i), N_points, 'return', i);
                tj_stabilize(:, :, i) = create_joint_traj(tj_support(:, :, i), N_points, 'stabilizing', i);
                pause
            end
            % Group 1 -> legs 1,3,5; Group 2 -> legs 2,4,6
            execute_trajectory(serial_obj, tj_positioning(:, :, group1), [], ...
                'positioning', 'none', N_points);
            execute_trajectory(serial_obj, tj_support(:, :, group1), tj_positioning(:, :, group2), ...
                'execution', 'positioning', N_points);
            for i=1:4
                execute_trajectory(serial_obj, tj_return(:, :, group1), tj_support(:, :, group2), ...
                    'return', 'execution', N_points);
                execute_trajectory(serial_obj, tj_support(:, :, group1), tj_return(:, :, group2), ...
                    'execution', 'return', N_points);
            end
            execute_trajectory(serial_obj, tj_stabilize(:, :, group1), tj_support(:, :, group2), ...
                'stabilizing', 'execution', N_points);
            execute_trajectory(serial_obj, [], tj_stabilize(:, :, group2), ...
                'none', 'stabilizing', N_points);
        % ----- state rotate_left ---------
        case 'rotate_left'
        

        % ----- state rotate_right --------
        case 'rotate_right'

        % ----- state steady --------------
        case 'steady'
        
            
        % ----- Default case --------------
        otherwise
            next_state = 'wait_for_input';
        
        
    
    end

    current_state = next_state;
end




