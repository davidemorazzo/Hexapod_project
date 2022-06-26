clc
clear all 
close all

%% Create the Arduino object


%% Create an ultrasonic sensor object with trigger pin D12 and echo pin D13.
%  Create a servo object for the servo connected to pin D3. 
sensor = ultrasonic(a,'D12', 'D13');%Create un ultraasonic sensor, with pins to be checked depending on your board
servo_motor = servo(a, 'D3');%Create a servo_motor object, with pin to be verified(IT MUST BE THE MOTOR ON THE HEAD)

%% Rotate the servo motor from 0 to 180 degrees.
%  Every time the motor rotates, determine the distance of any obstacles
%  via the bounceback time of the ultrasonic ping. Take two measurements
%  and average them for accuracy. Record the angle and the distance (in cm)
i = 1;
table = zeros(180,2);
for theta = 0 : 180
    arduino_head_pos(serial_obj,theta);
    dist1 = arduino_ultrasonic(serial_obj)%write position of the motor and read distance twice in order to create a 
    %more accurate mapping of the surroundings
    pause(.04);
    dist2 = arduino_ultrasonic(serial_obj)
    dist = (dist1+dist2)/2;%do the average value
    table(i,1) = (i-1);%put in the table the degree(from 1 to 180) and the rounded distance
    table(i,2) = round(dist * 100,2);
    i = i + 1;
end
%  Rotate the servo motor from 180 to 0 degrees. Replace the values in the
%  table with the average of the clockwise and counterclockwise scans to
%  improve the accuracy of the map.
j = 1; %here you do the same thing but rotating in the other direction
for theta = 180 : -1 : 0
    arduino_head_pos(serial_obj,theta);
    dist1 = arduino_ultrasonic(serial_obj)
    pause(.04);
    dist2 = arduino_ultrasonic(serial_obj)
    dist = (dist1+dist2)/2;
    table(i-j,2) = (table(i-j,2) + round(dist * 100,2))/2;
    j = j + 1;
end

%% Make a polar plot of the distance data to display the map. 
%  Limit the theta values between 0 and pi
polarplot (table(:,1)*pi/180, table (:,2));%just plotting the results into a radar form
title('Map of the Environment');
thetalim([0 180]);
grid on;