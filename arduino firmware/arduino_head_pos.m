function arduino_head_pos(serial_obj, position)
    % - serial_obj: serial object representing the connection
    % - position:   Scalar value representing the head servo angle
    %               The value of the angle is a number from 0 to 180.
        
    % Send command_id so arduino understand the following data. Command '2'
    % means head servomotor position
    serial_obj.write(4, 'int8');
    
    % Write the angle value on the seria as a byte
    serial_obj.write(uint8(position), 'char')
end