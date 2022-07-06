clear
close 
clc
format compact
rng('shuffle')
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
                "\n5 -> Steady" + ...
                "\n6 -> Ultrasonic" + ...
                "\n7 -> Avoid obstacle" + ...
                "\n8 -> MPU reading\n");
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
                case 6
                    next_state='ultrasonic';
                case 7
                    next_state='avoid_obstacle';
                case 8
                    next_state = 'mpu_reading';
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
            rotation_angle = 22.5; % 90 degrees
            
            N_points = 16;
            move_rotation(legs, serial_obj, rotation_angle, N_points);
            next_state = 'wait_for_input';

        % ----- State rotate_right --------

        case 'rotate_right'
            rotation_angle = -22.5;
            N_points = 16;
            move_rotation(legs, serial_obj, rotation_angle, N_points);
            next_state = 'wait_for_input';

        % ----- State steady --------------
        
        case 'steady'
            arduino_servo_pos(serial_obj, 90*ones(6, 1), 1);
            arduino_servo_pos(serial_obj, 90*ones(6, 1), 2);
            next_state = 'wait_for_input';
        

        % ----- State read ultrasonic -------
        case 'ultrasonic'
            [table] = map_the_surroundings(serial_obj);
            figure
            polarplot (table(:,1)*pi/180, table (:,2)); %just plotting the results into a radar form
            title('Map of the Environment');
            thetalim([20 160]);
            grid on;
            pause
            next_state = 'wait_for_input';

        case 'avoid_obstacle'
            arduino_head_pos(serial_obj,90);
            while(1)

                distance=arduino_ultrasonic(serial_obj);
                step = 3; % step length
                N_points = 16; % number of points in the trajectory discretization
                if distance>25
                    N_routines = 2; % number of moving routines to be executed
                    theta_a = 0; % direction of the hexapod [deg] (0 -> forward, 90 -> right)
                    move_linear(legs, serial_obj, step, theta_a, N_routines, N_points);
                else
                    N_points = 16;
                    rotation_sign = sign(rand-0.5); % definition of rotation direction via probabilistic approach
                    rotation_angle = 22.5;
                    move_rotation(legs, serial_obj, rotation_angle, N_points);

                    rotation_angle = rotation_sign*(11.25+sqrt(5)*randn);
                    % possible changes: (comment the above one to test)
                    % rotation_angle = rotation_sign*(11.25+sqrt(8)*randn);
                    % rotation_angle = rotation_sign*22.5*rand;
                    move_rotation(legs, serial_obj, rotation_angle, N_points);
                end
            end

%             while(1)
% 
%                 distance=arduino_ultrasonic(serial_obj);
%                 step = 3; % step length
%                 N_points = 16; % number of points in the trajectory discretization
%                 if distance>25
%                     N_routines = 3; % number of moving routines to be executed
%                     theta_a = 0; % direction of the hexapod [deg] (0 -> forward, 90 -> right)
%                     move_linear(legs, serial_obj, step, theta_a, N_routines, N_points);
%                 else
%                     N_routines = 6; % number of moving routines to be executed
%                     theta_a = 90; % direction of the hexapod [deg] (0 -> forward, 90 -> right)
%                     move_linear(legs, serial_obj, step, theta_a, N_routines, N_points);
%                 end
%             end
            next_state = 'wait_for_input';
        case 'mpu_reading'
            stabilize(serial_obj);
            next_state = 'wait_for_input';
        otherwise
            next_state = 'wait_for_input';
    end

    current_state = next_state;
end




