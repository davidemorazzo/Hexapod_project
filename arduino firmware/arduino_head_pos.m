function arduino_head_pos(serial_obj, position)
    serial_obj.write(4, 'int8'); % write command_id
    serial_obj.write(uint8(position), 'char')
end