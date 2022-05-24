function q = legTrajectory(legObj, d, u, N_points)

% legObj    -> SerialLink representing the leg
% d         -> Step length
% u         -> Step direction
% n_points  -> Points in the trajectory

% determination of the stable point
q_stable = [1, 0, pi/2];        % stable configuration
stable_point = legObj.fkine(q_stable);
P = stable_point.t;

% starting point and end point
P0 = - d/2 * u + P;    
P1 = + d/2 * u + P;
Ps = SE3(P0);
Pe = SE3(P1);

% Inverse kinematic trajectory
M = [1 1 1 0 0 0];
tj_points = ctraj(Ps, Pe, N_points);
joint_points = legObj.ikine(tj_points, 'mask', M, 'q0', q_stable, 'tol', 0.2);
joint_points(:,2) = joint_points(:,2) + pi/2;

q = joint_points;
return
