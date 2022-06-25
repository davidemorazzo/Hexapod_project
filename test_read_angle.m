clear
clc
serial_obj = serialport("COM15", 57600);
serial_obj.configureTerminator("CR/LF")
pause(1)
serial_obj.readline()
pause(1)
while(1)
    [angleX angleY] = arduino_read_angle(serial_obj)
    pause(0.5)
end
clear
