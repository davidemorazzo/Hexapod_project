clear
clc
serial_obj = serialport('COM14', 57600);
serial_obj.configureTerminator("CR/LF")
pause(1);
handshake = serial_obj.readline()
serial_obj.flush("input");
serial_obj.write(1, 'int8');
command = serial_obj.read(1, "uint8")
data = uint8(40)
serial_obj.flush("input");
serial_obj.write(data, 'uint8');
serial_obj.read(1, 'uint8')
% serial_obj.read(1, 'uint8')
% for j=1:length(data)
%     for i=1:6
%         data_writing = uint8(data(j))
% %         serial_obj.write(uint8(data(j)), 'uint8');
%         serial_obj.write(data, 'char');
%         pos = serial_obj.read(1, 'uint8')
%     end
% end