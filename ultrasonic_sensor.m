clc
clear all
close all

arduino=serial('COM9','BaudRate',9600);

a=arduino('COM7','Uno','libraries','Ultrasonic');
%u=ultrasonic(a,'A2','A3');
u=ultrasonic(a,'D11','D12');

distance=readDistance(a)