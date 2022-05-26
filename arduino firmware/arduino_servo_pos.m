function arduino_servo_pos(serial_obj, positions)
% Send command_id
serial_obj.write(1, 'int16');
cmd_id = serial_obj.read(1, 'int8');

% Provide a vector of angles in degrees
for i=1:12
    angle = uint8(positions(i));
    serial_obj.write(angle, 'char');
end

% Send termination
% serial_obj.write('\n', 'char');


end