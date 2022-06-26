function OK = map_the_surroundings(serial_obj)
%UNTITLED2 Summary of this function goes here
%   Detailed explanation goes here
    
    %% Create an ultrasonic sensor object with trigger pin D12 and echo pin D13.
    %  Create a servo object for the servo connected to pin D3. 
    sensor = ultrasonic(arduino_object,'D12', 'D13');%Create un ultraasonic sensor, with pins to be checked depending on your board
    servo_motor = servo(arduino_object, 'D3');%Create a servo_motor object, with pin to be verified(IT MUST BE THE MOTOR ON THE HEAD)

    %% Rotate the servo motor from 0 to 180 degrees.
    %  Every time the motor rotates, determine the distance of any obstacles
    %  via the bounceback time of the ultrasonic ping. Take two measurements
    %  and average them for accuracy. Record the angle and the distance (in cm)
    i = 1;
    table = zeros(180,2);%a clean table:first column is the degree, second column is the distance measured
    for theta = 0 : 1/180 : 1%theta goes from 0 to 180 degree with step 1
        writePosition(servo_motor, theta);%move position of the motor 
        dist1 = readDistance(sensor);%read distance twice in order to create a 
        %more accurate mapping of the surroundings
        pause(.04);
        dist2 = readDistance(sensor);
        dist = (dist1+dist2)/2;%do the average value
        table(i,1) = (i-1);%put in the table the degree value(from 0 to 180) 
        table(i,2) = round(dist * 100,2);%put in the table the rounded value of the distance measured
        i = i + 1;
    end
    %  Rotate the servo motor from 180 to 0 degrees. Replace the values in the
    %  table with the average of the clockwise and counterclockwise scans to
    %  improve the accuracy of the map.
    j = 1; %here you do the same thing but rotating in the other direction
    for theta = 1 : -1/180 : 0 %so go from 180 to 0 with a step of 1 degree
        writePosition(servo_motor, theta);%move the position of the servomotor
        dist1 = readDistance(sensor);%read the first time
        pause(.04);
        dist2 = readDistance(sensor);%read the secon time
        dist = (dist1+dist2)/2;%do the average
        table(i-j,2) = (table(i-j,2) + round(dist * 100,2))/2;%do the average of the two measurements, from 0 to 180 and from 180 to 0
        j = j + 1;
    end

    %% Make a polar plot of the distance data to display the map. 
    %  Limit the theta values between 0 and pi
    polarplot (table(:,1)*pi/180, table (:,2));%just plotting the results into a radar form, first column is number of degree, second value measured
    title('Map of the Environment');
    thetalim([0 180]);
    grid on;

    OK=1;
end