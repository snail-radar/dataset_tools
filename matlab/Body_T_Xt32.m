
function T = Body_T_Xt32()
R_body_xt32 = [0 -1 0; 1 0 0; 0 0 1];
p_body_xt32 = [0; 0; 0];
T = [R_body_xt32 p_body_xt32; 0, 0, 0, 1];
end
