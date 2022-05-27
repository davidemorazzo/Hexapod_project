% Create serial and communicate to arduino
clear 
close all
clc
format compact

serial_obj = serialport('COM6', 57600);
serial_obj.configureTerminator("CR/LF")
pause(1);
handshake = serial_obj.readline()

data = 0:0.05:180;
for i = 1:length(data)
    arduino_servo_pos(serial_obj, data(i) * ones(1,12));
    arduino_head_pos(serial_obj, data(i));
end


pause(0.5)
clear serial_obj
