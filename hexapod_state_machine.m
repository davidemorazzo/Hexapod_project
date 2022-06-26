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

load angle.mat
com_port = 'COM15';
serial_obj = serialport(com_port, 57600);
serial_obj.configureTerminator("CR/LF")
pause(1);
serial_obj.readline()
arduino_servo_pos(serial_obj, 90*ones(6, 1), 1);
arduino_servo_pos(serial_obj, 90*ones(6, 1), 2);

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
P0 = zeros(6, 3, 1); % initial point of the trajectory
P1 = zeros(6, 3, 1); % end point of the trajectory
group1 = [1 3 5]; % leg group 1
group2 = [2 4 6]; % leg group 2

% State machine
while true
    switch current_state
        
        % ----- State wait_for_input -------

        case'wait_for_input'
            user_input = input("Insert command\n1 -> Walk forward" + ...
                "\n2 -> Walk backward" + ...
                "\n3 -> Rotate right" + ...
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

        % ----- State walk_forward -----
        
        case 'walk_forward'
            step = 3; % step length
            theta_a = 0; % direction of the hexapod [deg] (0 -> forward, 90 -> right)
            N_routines = 5; % number of moving routines to be executed
            N_points = 16; % number of points in the trajectory discretization
            move_linear(legs, serial_obj, step, theta_a, N_routines, N_points);
            next_state = 'wait_for_input'; % Next state evaluation
        
        % ----- State walk_backward -------
        
        case 'walk_backward'
            step = 3;
            theta_a = 180;
            N_routines = 5;
            N_points = 16;
            move_linear(legs, serial_obj, step, theta_a, N_routines, N_points);
            next_state = 'wait_for_input';
        
        % ----- State rotate_left ---------

        case 'rotate_left'
            rotation_angle = 22.5;
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
            for i=1:2
                execute_trajectory(serial_obj, tj_return(:, :, group1), tj_support(:, :, group2), ...
                    'return', 'execution', N_points);
                execute_trajectory(serial_obj, tj_support(:, :, group1), tj_return(:, :, group2), ...
                    'execution', 'return', N_points);
            end
            execute_trajectory(serial_obj, tj_stabilize(:, :, group1), tj_support(:, :, group2), ...
                'stabilizing', 'execution', N_points);
            execute_trajectory(serial_obj, [], tj_stabilize(:, :, group2), ...
                'none', 'stabilizing', N_points);
        next_state = 'wait_for_input';

        % ----- State rotate_right --------

        case 'rotate_right'
            rotation_angle = -22.5;
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
            for i=1:2
                execute_trajectory(serial_obj, tj_return(:, :, group1), tj_support(:, :, group2), ...
                    'return', 'execution', N_points);
                execute_trajectory(serial_obj, tj_support(:, :, group1), tj_return(:, :, group2), ...
                    'execution', 'return', N_points);
            end
            execute_trajectory(serial_obj, tj_stabilize(:, :, group1), tj_support(:, :, group2), ...
                'stabilizing', 'execution', N_points);
            execute_trajectory(serial_obj, [], tj_stabilize(:, :, group2), ...
                'none', 'stabilizing', N_points);
        next_state = 'wait_for_input';

        % ----- State steady --------------
        
        case 'steady'
        
        % ----- State read ultrasonic -------

        arduino_servo_pos(serial_obj,[45 90 135 90 90 90], 1);
        arduino_servo_pos(serial_obj, [90 90 45 90 135 90], 2);
        l1 = 3.75;
        base = 8.7;
        tool_off = 2.25;
        l2 = 6.05;
        a = 2*l1+base+tool_off;
        while(1)
            [angleX, angleY] = arduino_read_angle(serial_obj);
            if(angleX<=1)
                angleX
            end
            if(angleX>1)
                arg = (l2-a*sind(angleX))/l2;
                q = acosd(arg)
                arduino_servo_pos(serial_obj, [45 90-q 135 90-q 90 90], 1)
                arduino_servo_pos(serial_obj, [90 90-q 45 90 135 90], 2);
            end
%             else
%                 arduino_servo_pos(serial_obj, [45 90 135 90 90 90+q], 1)
%                 arduino_servo_pos(serial_obj, [90 90 45 90+q 135 90+q], 2);
%             end
            pause(0.01)
        end
        next_state = 'wait_for_input';
        case 'ultrasonic'
            
        % ----- Default case --------------

        otherwise
            next_state = 'wait_for_input';
        
        
    end

    current_state = next_state;
end




