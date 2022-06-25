clear all
close all

connection = tcpclient('192.168.4.1',80);

while(1)
connection.write([5 90],'uint8')
pause(0.5)
end