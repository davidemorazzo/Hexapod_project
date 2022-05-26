% Create serial and communicate to arduino
clear 
close all
clc

serial_obj = serialport('COM6', 115200);
serial_obj.configureTerminator("CR/LF")
pause(1);
handshake = serial_obj.readline()


for i = 1:200
    data = i*0.9*ones(1,12);
    tic
    arduino_servo_pos(serial_obj, data);
    toc
    pause(0.05)
end



pause(0.5)
clear serial_obj
