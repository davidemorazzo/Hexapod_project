function [traj_saturated] = saturate_traj(traj, type, leg_index)
% saturation limits
saturation_a_max = 2/3*pi; 
saturation_a_min = pi/3;
saturation_b_max_right = 17/18*pi;
saturation_b_min_right = pi/4;
saturation_b_min_left = pi/18;
saturation_b_max_left = 3*pi/4;

if(leg_index==4 || leg_index==5 || leg_index==6)
    if(strcmp(type,'rad')==1)
        traj_saturated(:, 1) = min(saturation_a_max, max(saturation_a_min, traj(:, 1))); % motor a 
        traj_saturated(:, 2) = min(saturation_b_max_right, max(saturation_b_min_right, traj(:, 2))); % motor b
    elseif(strcmp(type,'deg')==1)
        traj_saturated(:, 1) = min(rad2deg(saturation_a_max), max(rad2deg(saturation_a_min), traj(:, 1))); % motor a 
        traj_saturated(:, 2) = min(rad2deg(saturation_b_max_right), max(rad2deg(saturation_b_min_right), traj(:, 2))); % motor b
    else
        traj_saturated = [];
    end
else
    if(strcmp(type,'rad')==1)
        traj_saturated(:, 1) = min(saturation_a_max, max(saturation_a_min, traj(:, 1))); % motor a 
        traj_saturated(:, 2) = min(saturation_b_max_left, max(saturation_b_min_left, traj(:, 2))); % motor b
    elseif(strcmp(type,'deg')==1)
        traj_saturated(:, 1) = min(rad2deg(saturation_a_max), max(rad2deg(saturation_a_min), traj(:, 1))); % motor a 
        traj_saturated(:, 2) = min(rad2deg(saturation_b_max_left), max(rad2deg(saturation_b_min_left), traj(:, 2))); % motor b
    else
        traj_saturated = [];
    end
end
end