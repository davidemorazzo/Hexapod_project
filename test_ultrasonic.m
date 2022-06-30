clear
clc
com_port = 'COM15';
serial_obj = serialport(com_port, 57600);
serial_obj.configureTerminator("CR/LF")
pause(1);
serial_obj.readline()
arduino_servo_pos(serial_obj, 90*ones(6, 1), 1);
arduino_servo_pos(serial_obj, 90*ones(6, 1), 2);
while(1)
    dist = arduino_ultrasonic(serial_obj)
    pause(0.5)
end