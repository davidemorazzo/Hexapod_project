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
com_port = '';
a = arduino(com_port, 'uno',...
    'Libraries', {'Servo' 'Ultrasonic' 'I2C'},...
    'BaudRate', 128000);
s1_1 = servo(a, 'D3');
s1_2 = servo(a, 'D2');
s2_1 = servo(a, 'D5');
s2_2 = servo(a, 'D4');
s3_1 = servo(a, 'D7');
s3_2 = servo(a, 'D6');
s4_1 = servo(a, 'D13');
s4_2 = servo(a, 'D12');
s5_1 = servo(a, 'D11');
s5_2 = servo(a, 'D10');
s6_1 = servo(a, 'D9');
s6_2 = servo(a, 'D8');
writePosition(s1_1, normalize_angle(angle_1_1, 'deg'));
writePosition(s1_2, normalize_angle(angle_1_2, 'deg'));
writePosition(s2_1, normalize_angle(angle_2_1, 'deg'));
writePosition(s2_2, normalize_angle(angle_2_2, 'deg'));
writePosition(s3_1, normalize_angle(angle_3_1, 'deg'));
writePosition(s3_2, normalize_angle(angle_3_2, 'deg'));
writePosition(s4_1, normalize_angle(angle_4_1, 'deg'));
writePosition(s4_2, normalize_angle(angle_4_2, 'deg'));
writePosition(s5_1, normalize_angle(angle_5_1, 'deg'));
writePosition(s5_2, normalize_angle(angle_5_2, 'deg'));
writePosition(s6_1, normalize_angle(angle_6_1, 'deg'));
writePosition(s6_2, normalize_angle(angle_6_2, 'deg'));

%% Robot's leg creation
legs = [createLeg(1) createLeg(2) createLeg(3) createLeg(4) createLeg(5) createLeg(6)]';
%% State machine
current_state = 'wait_for_input';
next_state = '';

% State machine
while true
    switch current_state
        % State wait_for_input
        case'wait_for_input'
            user_input = input("Insert command\n1 -> Walk forward\n2 -> Walk backward\n");
            user_input = int8(user_input);
            switch user_input
                case 1
                    next_state = 'walk_forward';
                case 2
                    next_state = 'walk_backward';
                otherwise
                    next_state = 'wait_for_input';
            end

        % state walk_forward
        case 'walk_forward'
            
            next_state = 'wait_for_input';
            disp("Walking forward yee")
        case 'walk_backward'
        % Default case
        otherwise
            next_state = 'wait_for_input';
    
    end

    current_state = next_state;
end




