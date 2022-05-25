clear
clc
close all
load angle.mat
legs = [createLeg(1) createLeg(2) createLeg(3) createLeg(4) createLeg(5) createLeg(6)]';

step = 3;
theta_a = 0;
N_points = 20;

[direct_traj1, return_traj1] = legTrajectory(legs, step, theta_a, N_points, 6);