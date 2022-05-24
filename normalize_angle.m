function angle_n = normalize_angle(angle, type)
    if(strcmp(type,'deg'))
        angle_n = angle/180;
    else
        angle_n = angle/2*pi;
    end
end