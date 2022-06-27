clc 
clear
com_port = 'COM15';
serial_obj = serialport(com_port, 57600);
serial_obj.configureTerminator("CR/LF")
pause(1);
serial_obj.readline()
arduino_servo_pos(serial_obj,[45 90 135 90 90 90], 1);
arduino_servo_pos(serial_obj, [90 90 45 90 135 90], 2);
l1 = 3.75;
base = 8.7;
tool_off = 2.25;
l2 = 6.05;
a = 2*l1+base+tool_off;

% theta is the angle describing the plane inclination
% angleX is the angle read by the MPU6050
% q is the incremental angle to which the 'second legs' are subjected in order to stabilize the hexapod
% N is a big number in order to allow us to measure angles for a sufficient time equal to N*0.5 seconds
theta_old = 0;
threshold = 3;
K = 0.2;

% Possible paramters: K=0.5 (applied to q), threshold=2; K=0.2 (applied to
% theta_new), threshold=3
while(1)
    [angleX, angleY] = arduino_read_angle(serial_obj);
    theta_new=theta_old+angleX;
    theta_abs = K*abs(theta_new);
    arg = (l2-a*sind(theta_abs))/l2;
    q = acosd(arg);

    if (theta_new>threshold && theta_new>theta_old)
        arduino_servo_pos(serial_obj, [45 90-q 135 90-q 90 90], 1);
        arduino_servo_pos(serial_obj, [90 90-q 45 90 135 90], 2);
    elseif (theta_new<-threshold && theta_new<theta_old)
        arduino_servo_pos(serial_obj, [45 90 135 90 90 90+q], 1);
        arduino_servo_pos(serial_obj, [90 90 45 90+q 135 90+q], 2);
%     elseif abs(theta_new)<threshold
%         disp('Stable plane');
%         arduino_servo_pos(serial_obj, [45 90 135 90 90 90], 1);
%         arduino_servo_pos(serial_obj, [90 90 45 90 135 90], 2);
    end
    pause(0.01)
end
