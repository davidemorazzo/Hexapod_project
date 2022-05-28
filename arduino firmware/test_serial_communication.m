clear
clc
serial_obj = serialport('COM11', 30000);
serial_obj.configureTerminator("CR/LF")
handshake = serial_obj.readline()
while(1)
    str = serial_obj.readline()
    sum = int8(str2num(str))+1;
    serial_obj.writeline(int2str(sum));
%     serial_obj.writeline('1');
    pause(0.001)
end