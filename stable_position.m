function [] = stable_position(servos)

load angle.mat
writePosition(servos(1), normalize_angle(angles(1).a, 'deg'));
writePosition(servos(2), normalize_angle(angles(1).b, 'deg'));
writePosition(servos(3), normalize_angle(angles(2).a, 'deg'));
writePosition(servos(4), normalize_angle(angles(2).b, 'deg'));
writePosition(servos(5), normalize_angle(angles(3).a, 'deg'));
writePosition(servos(6), normalize_angle(angles(3).b, 'deg'));
writePosition(servos(7), normalize_angle(angles(4).a, 'deg'));
writePosition(servos(8), normalize_angle(angles(4).b, 'deg'));
writePosition(servos(9), normalize_angle(angles(5).a, 'deg'));
writePosition(servos(10), normalize_angle(angles(5).b, 'deg'));
writePosition(servos(11), normalize_angle(angles(6).a, 'deg'));
writePosition(servos(12), normalize_angle(angles(6).b, 'deg'));

end
