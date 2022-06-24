clear
clc
serial_obj = serialport("COM15", 57600);
serial_obj.configureTerminator("CR/LF")
pause(1)
serial_obj.readline()
while(1)
    angle = arduino_read_angle(serial_obj, 'z')
    pause(1)
end
clear
