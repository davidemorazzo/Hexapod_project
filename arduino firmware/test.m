clear 
close all
clc
format compact

%% Create Serial object to communicate to arduino
% Important set BaudRate to the same specified inside arduino firmware
serial_obj = serialport('COM6', 57600);
serial_obj.configureTerminator("CR/LF")
pause(1);

% Response from arduino when the connection is established
handshake = serial_obj.readline()


%% Test using the functions
% data = 0:0.05:180;
data = 90+90.*sind(1:1:720*2);
for i = 1:length(data)
    arduino_servo_pos(serial_obj, data(i) * ones(1,12));
    arduino_head_pos(serial_obj, data(i));
    pause(0.005)
end


pause(0.5)
%% Important to close the serial port
clear serial_obj
