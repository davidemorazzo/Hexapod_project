function angle = arduino_read_angle(serial_obj, type)
    serial_obj.flush("input");
    if(strcmp(type, 'z')==1)
        serial_obj.write(4, 'uint8');
        angle = serial_obj.read(1, "single");
    end
end