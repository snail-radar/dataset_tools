function T = Ars548ToBody(date)
p_body_ars548 = [0; 0; 0.07];
R_body_ars548 = eye(3); % nominal
if date < 20231104 % no ars548
    T = [];
elseif date < 20231105.5 % that morning tilted backward
    % to convert the ars548 frame to the body frame,
    % first rotate about the ars548 frame y-axis by 5 degree,
    % then rotate about the new z-axis by 3 degree.
    R_body_ars548 = R3d(3) * R2d(5);
    T = [R_body_ars548 p_body_ars548; 0, 0, 0, 1];
else % that afternoon corrected pitch somewhat
    % to convert the ars548 frame to the body frame,
    % first rotate about the ars548 frame x-axis by 1 degree,
    % then rotate about the new z-axis by 3 degree.
    R_body_ars548 = R3d(3) * R2d(1);
    T = [R_body_ars548 p_body_ars548; 0, 0, 0, 1];
end
end
