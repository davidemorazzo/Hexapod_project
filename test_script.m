clear
clc
close all
load angle.mat
legs = [createLeg(1) createLeg(2) createLeg(3) createLeg(4) createLeg(5) createLeg(6)]';

step = 9;
theta_a = 0;
N_points = 20;
leg_index = 5;
visualize = 1;

[direct_traj1, return_traj1] = legTrajectory(legs, step, theta_a, N_points, leg_index, visualize);