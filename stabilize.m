function [] = stabilize(serial_obj)

arduino_servo_pos(serial_obj,[90 90 135 90 90 90], 1);
arduino_servo_pos(serial_obj, [90 90 45 90 135 90], 2);

l1 = 3.75;
base = 8.7;
tool_off = 2.25;
l2 = 6.05;
a = 2*l1+base+tool_off;

% theta is the angle describing the plane inclination
% angleX is the angle read by the MPU6050
% q is the incremental angle to which the 'second legs' are subjected in order to stabilize the hexapod

theta_old = 0; % inclined plane angle (start: 0)
threshold = 4.5; % angle threshold for the inclined plane to decide to take action
K = 0.2; % controller gain

while(1)
    [angleX, ~] = arduino_read_angle(serial_obj)
    theta_new = theta_old+angleX-0.5
    
    if abs(theta_new)<threshold
        theta_new=angleX;
    else
        theta_new=theta_new;
    end
    theta_abs = K*abs(theta_new);
    arg = (l2-a*sind(theta_abs))/l2;
    q = acosd(arg);

    if (theta_new>threshold)
        arduino_servo_pos(serial_obj, [45 90-q 135 90-q 90 90], 1);
        arduino_servo_pos(serial_obj, [90 90-q 45 90 135 90], 2);

    elseif (theta_new<-threshold)
        arduino_servo_pos(serial_obj, [45 90 135 90 90 90+q], 1);
        arduino_servo_pos(serial_obj, [90 90 45 90+q 135 90+q], 2);

    elseif (abs(theta_new)<threshold && angleX<2)
        arduino_servo_pos(serial_obj, [45 90 135 90 90 90], 1);
        arduino_servo_pos(serial_obj, [90 90 45 90 135 90], 2);

    end
    theta_old = theta_new;
    pause(0.02)
end
end