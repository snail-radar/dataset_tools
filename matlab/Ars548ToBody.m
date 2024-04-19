function T = Ars548ToBody(date, comment)
if nargin < 2
    comment = '';
end
p_body_ars548 = [0; 0; 0.07];
if comment == 'nominal':
    R_body_ars548 = eye(3);
else
    if date < 20231105.5
        R_body_ars548 = R2deg(5) * R3deg(3);
    else
        R_body_ars548 = R2deg(1) * R3deg(3);
    end
end

T = [R_body_ars548; p_body_ars548];
end
