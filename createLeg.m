function leg = createLeg(index)
% Leg index

%% Create robot
% Link length
l0 = 5;
l1 = 3.75;
l2 = 6.05;
o_1 = -3.1;
o_2 = 2.25;

% Create joints
base = Prismatic('qlim', 4, 'a', l0);
leg1 = Revolute('a',l1,'alpha', -pi/2, 'd',o_1);
leg2 = Revolute('a',l2,'alpha', pi, 'd',0);
% Set base frame
base_xyz = SE3();
if index == 1
    base_xyz.t = [0 0 7.1500];
elseif index == 2
    base_xyz.t = [0 0 7.1500];
elseif index == 3
    base_xyz = SE3().Rz(pi);
    base_xyz.t = [0 0 7.1500];
end
% Create robot object
leg = SerialLink([base, leg1, leg2],...
    'name', 'hexapod', ...
    'base', base_xyz, ...
    'tool', SE3([0 o_2 0]'));