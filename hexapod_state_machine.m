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

% - rotate_rigth
%   - right_legs
%   - left_legs


current_state = 'wait_for_input'
next_state = '';


% State machine
while true
    switch current_state
        % State wait_for_input
        case'wait_for_input'
            value = input("Insert command");
            if value == 1
                next_state = 'walk_forward';
            elseif value == 2
                next_state = 'walk_backward';
            end
    
    
        % state walk_forward
        case 'walk_forward'
            next_state = 'wait_for_input';
    
    
        % Default case
        otherwise
            next_state = 'wait_for_input';
    
    end

    current_state = next_state;
end




