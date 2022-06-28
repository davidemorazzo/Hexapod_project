function [distance]=arduino_ultrasonic(serial_obj)
        
        serial_obj.write(3,"uint8");

        

        distance=serial_obj.read(1,"float");

end