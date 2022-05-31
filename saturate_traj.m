function [traj_saturated] = saturate_traj(traj, type)
% saturation limits
saturation_a_max = 2/3*pi; 
saturation_a_min = pi/3;
saturation_b_max = 3/4*pi;
saturation_b_min = pi/4;

if(strcmp(type,'rad')==1)
    traj_saturated(:, 1) = min(saturation_a_max, max(saturation_a_min, traj(:, 1))); % motor a 
    traj_saturated(:, 2) = min(saturation_b_max, max(saturation_b_min, traj(:, 2))); % motor b
elseif(strcmp(type,'deg')==1)
    traj_saturated(:, 1) = min(rad2deg(saturation_a_max), max(rad2deg(saturation_a_min), traj(:, 1))); % motor a 
    traj_saturated(:, 2) = min(rad2deg(saturation_b_max), max(rad2deg(saturation_b_min), traj(:, 2))); % motor b
else
    traj_saturated = [];
end