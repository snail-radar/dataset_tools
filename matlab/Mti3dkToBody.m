function T = Mti3dkToBody(comment)
if nargin < 1
    comment = 'refined';
end

% the mti3dk nominal frame is forward left up
% but its x axis is pointing to the right of the front direction
% to transfrom from the mti3dk frame to the body frame,
% we need to rotate the mti3dk frame by about 3 degrees about the z axis.
switch comment
    case 'manual'
        R = R3d(3);
        p = [0; 0; -0.18];
        T = [R p; 0, 0, 0, 1];
    case 'refined'
        T_Mti3dk_Xt32 = ...
            [-0.099438 -0.992995 -0.063813 -0.012713;
            0.994106 -0.101923  0.036942 -0.001295;
            -0.043187 -0.059763  0.997278  0.184497;
            0.000000  0.000000  0.000000  1.000000]; % estimated by lidar IMU init.
        T = Xt32ToBody() * inv(T_Mti3dk_Xt32);
    otherwise
        R = R3d(3);
        p = [0; 0; -0.18];
        T = [R p; 0, 0, 0, 1];
end
end
