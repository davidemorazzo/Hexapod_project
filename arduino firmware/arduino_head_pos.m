function arduino_head_pos(connection, position)
    % - serial_obj: serial object representing the connection
    % - position:   Scalar value representing the head servo angle
    %               The value of the angle is a number from 0 to 180.
        
    % Send command_id so arduino understand the following data. Command '2'
    % means head servomotor position
    connection.write([5 position], 'int8');
end