function [angleX, angleY] = arduino_read_angle(serial_obj)
    serial_obj.write(4, "uint8");
    angleX = serial_obj.read(1, "single");
    angleY = serial_obj.read(1, "single");
end