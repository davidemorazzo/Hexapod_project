clear 
close all
clc
format compact

%% Create Serial object to communicate to arduino
% Important set BaudRate to the same specified inside arduino firmware
serial_obj = serialport('COM15', 57600);
serial_obj.configureTerminator("CR/LF")
pause(1);

% Response from arduino when the connection is established
handshake = serial_obj.readline()


%% Test using the functions
% data = 0:0.05:180;
% data = 90+90.*sind(1:1:720*2);
data = [45:1:120 120:-1:45];
for i = 1:length(data)
    data(i)
    arduino_servo_pos(serial_obj, data(i) * ones(6, 1), 1);
    arduino_servo_pos(serial_obj, data(i) * ones(6, 1), 2);
    arduino_head_pos(serial_obj, data(i));
    pause(0.03)
end


pause(0.5)
%% Important to close the serial port
clear serial_obj
