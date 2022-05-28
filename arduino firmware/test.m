clear 
close all
clc
format compact

%% Create Serial object to communicate to arduino
% Important set BaudRate to the same specified inside arduino firmware
serial_obj = serialport('COM11', 30000);
serial_obj.configureTerminator("CR/LF")
pause(1);

% Response from arduino when the connection is established
handshake = serial_obj.readline()
% serial_obj.writeline('15');
% pause(0.5)
% serial_obj.readline()

%% Test using the functions
% data = 0:0.05:180;
% data = floor(90+90.*sind(1:1:720*2));
% data = [45:1:135 135:-1:45];
% data = [0 90];
arduino_servo_pos(serial_obj, 0* ones(1,12));
% for i = 1:2
%     arduino_servo_pos(serial_obj, data(i) * ones(1,12));
% %     arduino_head_pos(serial_obj, data(i));
%     pause(0.001)
% end

% for i = 1:length(data)
%     arduino_servo_pos(serial_obj, data(i) * ones(1,12));
% %     arduino_head_pos(serial_obj, data(i));
%     pause(0.001)
% end


pause(0.5)
%% Important to close the serial port
clear serial_obj
