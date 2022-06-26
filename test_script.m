clc
clear all
close all

arduino_object = arduino('COM9', 'Uno', 'Libraries', {'Ultrasonic', 'Servo'});
map_the_surroundings(arduino_object)