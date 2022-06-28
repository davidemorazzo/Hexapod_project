clear all
close all

connection = tcpclient('192.168.4.1',80)

% while(1)
%     % Write head position
%     connection.write([5 30],'uint8');
    connection.write([1 45*ones(1,6)],'uint8');
    databuf = connection.read(6, "uint8")
%     pause(0.5)
%     connection.write([1 120 120 120 120 120 120],'uint8');
%     pause(0.5)
%     connection.write([5 60],'uint8');
%     pause(0.5)
% end