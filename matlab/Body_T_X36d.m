function T = Body_T_X36d()
% For 0921/data5 which is very long (> 1000 secs) and rich in motion, 
% by aligning kissicp and x36d IMU data, we get
x36d_R_xt32 = [
-0.001402106 -0.999967799 0.007901562;
0.999997759 -0.001414596 -0.001575345;
0.001586472 0.007899336 0.999967541];

body_R_xt32 = [0 -1 0; 1 0 0; 0 0 1];
body_R_x36d = body_R_xt32 * transpose(x36d_R_xt32);

body_p_x36d = [-0.06; 0; -0.16];
T = [body_R_x36d body_p_x36d; 0, 0, 0, 1];

end
