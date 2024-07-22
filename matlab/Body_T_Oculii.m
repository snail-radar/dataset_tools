function T = Body_T_Oculii(comment)
% The transform of the oculii point cartesian frame (forward left up) relative to the body frame.
if nargin < 1
    comment = 'refined';
end
switch comment
case 'manual'
    R_body_oculii = eye(3);
    p_body_oculii = [0.07; 0; -0.115];
    T = [R_body_oculii p_body_oculii; 0, 0, 0, 1];

otherwise % refined
    % the relative rotation is found by estimating relative rot of oculii relative to the x36d frame
    % using the oculii ego velocity estimated from radar_pcl2 messages and INS solution from inspvaxa data,
    % of 20240113/data5.
    R_x36d_oculii = [0.999829834123514	0.0183936527785399	-0.001405821470991;
    	-0.0183512882701082	0.999498727400647	0.0257977546176971;
        0.00187963171211191	-0.0257675660851007	0.999666194048132];
    T_body_x36d = Body_T_X36d();
    R_body_x36d = T_body_x36d(1:3, 1:3);
    R_body_oculii = R_body_x36d * R_x36d_oculii;
    p_body_oculii = [0.07; 0; -0.115];
    T = [R_body_oculii p_body_oculii; 0, 0, 0, 1];
end
end
