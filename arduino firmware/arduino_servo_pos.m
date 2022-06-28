function arduino_servo_pos(connection, positions, leg_group)
    % - serial_obj: serial object representing the connection
    % - positions:  vector with length=12 representing the servo positions.
    %               The order is [s1_1, s1_2, s2_1, s2_2, ...]
    %               The value of each angle is a number from 0 to 180.
    %
    % Note: Head servomotor is not implemented in this function
    
    
    % Send command_id so arduino understand the following data. Command '1'
    % means servomotor positions
    tic
    [sz1, sz2] = size(positions);
    
    if(sz2==1)
        if(leg_group==1)
            connection.write([1 positions'], 'uint8');
        elseif(leg_group==2)
            connection.write([2 positions'], 'uint8');
        end
    else
        if(leg_group==1)
            connection.write([1 positions], 'uint8');
        elseif(leg_group==2)
            connection.write([2 positions], 'uint8');
        end
    end
    t_effective = toc;
    pause(0.01-t_effective)
end