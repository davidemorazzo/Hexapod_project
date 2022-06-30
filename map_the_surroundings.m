function [table] = map_the_surroundings(serial_obj)

    
    %% Rotate the servo motor from 0 to 180 degrees.
%  Every time the motor rotates, determine the distance of any obstacles
%  via the bounceback time of the ultrasonic ping. Take two measurements
%  and average them for accuracy. Record the angle and the distance (in cm)
i = 20;
table = zeros(141,2);
for theta = 20 : 160
    arduino_head_pos(serial_obj,theta); %move the head
    dist1 = arduino_ultrasonic(serial_obj); %write position of the motor and read distance twice in order to create a 
    %more accurate mapping of the surroundings
    pause(.08);
    dist2 = arduino_ultrasonic(serial_obj);
    dist = (dist1+dist2)/2 %do the average value
    table(i,1) = (i-1);%put in the table the degree(from 1 to 180) and the rounded distance
    table(i,2) = round(dist/100,2);
%     if(dist>400)
%         table(i, 2) = table(i-1, 2);
%     end
    i = i + 1;
end
%  Rotate the servo motor from 180 to 0 degrees. Replace the values in the
%  table with the average of the clockwise and counterclockwise scans to
%  improve the accuracy of the map.
j = 20; %here you do the same thing but rotating in the other direction
for theta = 160 : -1 : 20
    arduino_head_pos(serial_obj,theta);%move the head 
    dist1 = arduino_ultrasonic(serial_obj);
    pause(.08);
    dist2 = arduino_ultrasonic(serial_obj);
    dist = (dist1+dist2)/2
    table(i-j,2) = (table(i-j,2) + round(dist/100,2))/2;
    
%     if(dist>400)
%         table(i-j, 2) = table(i-j-1, 2);
%     end
    j = j + 1;
    
end
for i=1:141
    if(table(i, 2)>400)
        j=1;
        exit = false;
        while(~exit)
            if(table(i+j, 2)>400)
                j = j+1;
            else
                exit=true;
            end
        end
        table(i:i+j, 2) = ones(j, 1)*(table(i-1, 2)+table(i+j+1, 2))/2;
    end
end

%% Make a polar plot of the distance data to display the map. 
%  Limit the theta values between 0 and pi
% polarplot (table(:,1)*pi/180, table (:,2)); %just plotting the results into a radar form
% title('Map of the Environment');
% thetalim([20 160]);
% grid on;

end