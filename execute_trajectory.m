function execute_trajectory(serial_obj, traj_group1, traj_group2, state_group1, state_group2, N_points)

if(strcmp(state_group1, 'positioning') && strcmp(state_group2, 'none'))
    for i=1:N_points/2
        positions_group1 = [traj_group1(i, :, 1) traj_group1(i, :, 2) traj_group1(i, :, 3)];
        arduino_servo_pos(serial_obj, positions_group1, 1);
    end
elseif(strcmp(state_group1, 'none') && strcmp(state_group2, 'positioning'))
    for i=1:N_points/2
        positions_group2 = [traj_group2(i, :, 1) traj_group2(i, :, 2) traj_group2(i, :, 3)];
        arduino_servo_pos(serial_obj, positions_group2, 2);
    end
elseif(strcmp(state_group1, 'execution') && strcmp(state_group2, 'positioning'))
    for i=1:N_points/2
        positions_group1 = [traj_group1(i, :, 1) traj_group1(i, :, 2) traj_group1(i, :, 3)];
        arduino_servo_pos(serial_obj, positions_group1, 1);
        positions_group2 = [traj_group2(i, :, 1) traj_group2(i, :, 2) traj_group2(i, :, 3)];
        arduino_servo_pos(serial_obj, positions_group2, 2);
    end
    for i=(N_points/2+1):N_points
        positions_group1 = [traj_group1(i, :, 1) traj_group1(i, :, 2) traj_group1(i, :, 3)];
        arduino_servo_pos(serial_obj, positions_group1, 1);
    end
elseif(strcmp(state_group2, 'execution') && strcmp(state_group1, 'positioning'))
    for i=1:N_points/2
        positions_group1 = [traj_group1(i, :, 1) traj_group1(i, :, 2) traj_group1(i, :, 3)];
        arduino_servo_pos(serial_obj, positions_group1, 1);
        positions_group2 = [traj_group2(i, :, 1) traj_group2(i, :, 2) traj_group2(i, :, 3)];
        arduino_servo_pos(serial_obj, positions_group2, 2);
    end
    for i=(N_points/2+1):N_points
        positions_group2 = [traj_group2(i, :, 1) traj_group2(i, :, 2) traj_group2(i, :, 3)];
        arduino_servo_pos(serial_obj, positions_group2, 2);
    end
elseif(strcmp(state_group1, 'execution') && strcmp(state_group2, 'none'))
    for i=1:N_points
        positions_group1 = [traj_group1(i, :, 1) traj_group1(i, :, 2) traj_group1(i, :, 3)];
        arduino_servo_pos(serial_obj, positions_group1, 1);
    end
elseif(strcmp(state_group2, 'execution') && strcmp(state_group1, 'none'))
    for i=1:N_points
        positions_group2 = [traj_group2(i, :, 1) traj_group2(i, :, 2) traj_group2(i, :, 3)];
        arduino_servo_pos(serial_obj, positions_group2, 2);
    end
elseif((strcmp(state_group1, 'execution') && strcmp(state_group2, 'return')) || ...
        (strcmp(state_group2, 'execution') && strcmp(state_group1, 'return')))
    for i=1:N_points
        positions_group1 = [traj_group1(i, :, 1) traj_group1(i, :, 2) traj_group1(i, :, 3)];
        arduino_servo_pos(serial_obj, positions_group1, 1);
        positions_group2 = [traj_group2(i, :, 1) traj_group2(i, :, 2) traj_group2(i, :, 3)];
        arduino_servo_pos(serial_obj, positions_group2, 2);
    end
elseif(strcmp(state_group1, 'stabilizing') && strcmp(state_group2, 'execution'))
    for i=1:N_points/2
        positions_group1 = [traj_group1(i, :, 1) traj_group1(i, :, 2) traj_group1(i, :, 3)];
        arduino_servo_pos(serial_obj, positions_group1, 1);
        positions_group2 = [traj_group2(i, :, 1) traj_group2(i, :, 2) traj_group2(i, :, 3)];
        arduino_servo_pos(serial_obj, positions_group2, 2);
    end
    for i=(N_points/2+1):N_points
        positions_group2 = [traj_group2(i, :, 1) traj_group2(i, :, 2) traj_group2(i, :, 3)];
        arduino_servo_pos(serial_obj, positions_group2, 2);
    end
elseif(strcmp(state_group2, 'stabilizing') && strcmp(state_group1, 'execution'))
    for i=1:N_points/2
        positions_group1 = [traj_group1(i, :, 1) traj_group1(i, :, 2) traj_group1(i, :, 3)];
        arduino_servo_pos(serial_obj, positions_group1, 1);
        positions_group2 = [traj_group2(i, :, 1) traj_group2(i, :, 2) traj_group2(i, :, 3)];
        arduino_servo_pos(serial_obj, positions_group2, 2);
    end
    for i=(N_points/2+1):N_points
        positions_group1 = [traj_group1(i, :, 1) traj_group1(i, :, 2) traj_group1(i, :, 3)];
        arduino_servo_pos(serial_obj, positions_group1, 1);
    end
elseif(strcmp(state_group1, 'none') && strcmp(state_group2, 'stabilizing'))
    for i=1:N_points/2
        positions_group2 = [traj_group2(i, :, 1) traj_group2(i, :, 2) traj_group2(i, :, 3)];
        arduino_servo_pos(serial_obj, positions_group2, 2);
    end
elseif(strcmp(state_group2, 'none') && strcmp(state_group1, 'stabilizing'))
    for i=1:N_points/2
        positions_group1 = [traj_group1(i, :, 1) traj_group1(i, :, 2) traj_group1(i, :, 3)];
        arduino_servo_pos(serial_obj, positions_group1, 1);
    end
end

end