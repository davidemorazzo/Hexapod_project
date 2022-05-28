function arduino_servo_pos(serial_obj, positions)
    % - serial_obj: serial object representing the connection
    % - positions:  vector with length=12 representing the servo positions.
    %               The order is [s1_1, s1_2, s2_1, s2_2, ...]
    %               The value of each angle is a number from 0 to 180.
    %
    % Note: Head servomotor is not implemented in this function
    
    
    % Send command_id so arduino understand the following data. Command '1'
    % means servomotor positions
    serial_obj.writeline('1');
    
    % Write on the serial each angle in order as a byte.
    for i=1:12
%         angle = uint8(positions(i));
        serial_obj.writeline(int2str(positions(i)));
%         pause(0.001)
    end

end