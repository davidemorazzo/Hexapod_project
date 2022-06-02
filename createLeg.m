function leg = createLeg(index) % Leg index

%% Create robot
% Indexes -> CCW from robot head
% Link lengths (all parameters are in cm)

l0 = 0;
l1 = 3.75;
l2 = 6.05;
o_1 = -3.1;
o_2 = -2.25;
delta_x = 2;
delta_y = 6;
% index = 1;

prism_freedom = 2;

% Create joints
switch index
    case 1
        base = Prismatic('qlim', [0 1], 'a', l0, 'alpha', pi);
        leg1 = Revolute('a',l1,'alpha', pi-pi/2, 'd', -o_1, 'qlim', [0 pi], 'offset', 3/4*pi);
        leg2 = Revolute('a',l2,'qlim', [0 pi]);
        % Set base frame
        base_xyz = SE3();
        base_xyz.t = [-delta_x delta_y -o_1+l2-prism_freedom];
        leg = SerialLink([base, leg1, leg2],...
            'name', 'hexapod', ...
            'base', base_xyz, ...
            'tool', SE3([0 o_2 0]'));
    case 2
        base = Prismatic('qlim', [0 1], 'a', l0, 'alpha', pi);
        leg1 = Revolute('a',l1,'alpha', pi-pi/2, 'd',-o_1, 'qlim', [0 pi], 'offset', pi/2);
        leg2 = Revolute('a',l2,'qlim', [0 pi]);
        % Set base frame
        base_xyz = SE3();
        base_xyz.t = [-delta_x 0 -o_1+l2-prism_freedom];
        leg = SerialLink([base, leg1, leg2],...
            'name', 'hexapod', ...
            'base', base_xyz, ...
            'tool', SE3([0 o_2 0]'));
    case 3
        base = Prismatic('qlim', [0 1], 'a', l0, 'alpha', pi);
        leg1 = Revolute('a',l1,'alpha', pi-pi/2, 'd', -o_1, 'qlim', [0 pi], 'offset', pi/4);
        leg2 = Revolute('a',l2,'qlim', [0 pi]);
        % Set base frame
        base_xyz = SE3();
        base_xyz.t = [-delta_x -delta_y -o_1+l2-prism_freedom];
        leg = SerialLink([base, leg1, leg2],...
            'name', 'hexapod', ...
            'base', base_xyz, ...
            'tool', SE3([0 o_2 0]'));
    case 4
        base = Prismatic('qlim', [0 1], 'a', l0, 'alpha', pi);
        leg1 = Revolute('a',l1,'alpha', -pi/2, 'd', -o_1, 'qlim', [0 pi], 'offset', pi/4+pi/2);
        leg2 = Revolute('a',l2,'qlim', [0 pi], 'offset', pi);
        % Set base frame
        base_xyz = SE3().Rz(pi);
        base_xyz.t = [delta_x -delta_y -o_1+l2-prism_freedom];
        leg = SerialLink([base, leg1, leg2],...
            'name', 'hexapod', ...
            'base', base_xyz, ...
            'tool', SE3([0 -o_2 0]'));
    case 5
        base = Prismatic('qlim', [0 1], 'a', l0, 'alpha', pi);
        leg1 = Revolute('a',l1,'alpha', -pi/2, 'd', -o_1, 'qlim', [0 pi], 'offset', -pi/2);
        leg2 = Revolute('a',l2,'qlim', [0 pi], 'offset', pi);
        % Set base frame
        base_xyz = SE3();
        base_xyz.t = [delta_x 0 -o_1+l2-prism_freedom];
        leg = SerialLink([base, leg1, leg2],...
            'name', 'hexapod', ...
            'base', base_xyz, ...
            'tool', SE3([0 -o_2 0]'));
    case 6
        base = Prismatic('qlim', [0 1], 'a', l0, 'alpha', pi);
        leg1 = Revolute('a',l1,'alpha', -pi/2, 'd', -o_1, 'qlim', [0 pi], 'offset', -pi/2-pi/4);
        leg2 = Revolute('a',l2,'qlim', [0 pi], 'offset', pi);
        % Set base frame
        base_xyz = SE3();
        base_xyz.t = [delta_x delta_y -o_1+l2-prism_freedom];
        leg = SerialLink([base, leg1, leg2],...
            'name', 'hexapod', ...
            'base', base_xyz, ...
            'tool', SE3([0 -o_2 0]'));
    otherwise
        leg = SerialLink.empty;
end

% leg.plot([0 pi/2 pi/2])
% leg.fkine([0 pi/2 pi/2])
end