function p_body_ants = AntennasInBody(date)
if date < 20231025
    p_body_ants = [0; 0; 0];
elseif date < 20240105
    p_body_ants = [0, 0; 0, 0; 0, 0]; % first column for main antenna, second column for associate antenna. 
else
    p_body_ants = [0, 0; 0, 0; 0, 0];
end
end