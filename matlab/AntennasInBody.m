function p_body_ants = AntennasInBody(date)
if date < 20231006 % only an antenna on top of hesai pandar
    p_body_ants = [0; 0; 0];
elseif date < 20231018 % two antennas, one on top of pandar, the other on the left back side.
    % first column for main antenna, second column for associate antenna.
    % the x36d vehicle frame is right forward up
    % the sensor rig body frame is forward left up, so
    R_body_x36dv = [0, 1, 0; -1, 0, 0; 0, 0, 1];
    p_body_x36dv = [-0.06; 0; -0.16];
    p_x36dv_top = [0; 0.07; 0.35];
    p_x36dv_leftback = [-0.56; -0.43; 0.05];
    p_body_top = p_body_x36dv + R_body_x36dv * p_x36dv_top;
    p_body_leftback = p_body_x36dv + R_body_x36dv * p_x36dv_leftback;
    p_body_ants = [p_body_top p_body_leftback];
elseif date < 20231108 % two antennas, one on left side, the other on right side, both slightly tilt backward.
    left = [0.05; 0.445; -0.01];
    right = [0.05; 0.44; -0.01];
    p_body_ants = [left right];
else % two antennas, one on left side, the other on right side, both slightly tilt forward.
    left = [0.06; 0.445; -0.01];
    right = [0.06; 0.44; -0.01];
    p_body_ants = [left right];
end
end
