function arduino_servo_pos(serial_obj, positions, leg_group)
    % - serial_obj: serial object representing the connection
    % - positions:  vector with length=12 representing the servo positions.
    %               The order is [s1_1, s1_2, s2_1, s2_2, ...]
    %               The value of each angle is a number from 0 to 180.
    %
    % Note: Head servomotor is not implemented in this function
    
    
    % Send command_id so arduino understand the following data. Command '1'
    % means servomotor positions
    if(leg_group==1)
        serial_obj.write(1, 'uint8');
    elseif(leg_group==2)
        serial_obj.write(2, 'uint8');
    end
%     line = serial_obj.readline()
%     command = serial_obj.read(1, 'int8')
    % Write on the serial each angle in order as a byte.
    for i=1:6
        angle = uint8(positions(i));
        serial_obj.write(angle, 'char');
    end

end