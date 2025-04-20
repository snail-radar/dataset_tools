function T = Body_T_Zed2LeftCam(platform)
% A. manually measured
R_body_leftcam = [0, 0, 1; -1, 0, 0; 0, -1, 0];
p_body_leftcam = [0.125; 0.07; -0.195];
T = [R_body_leftcam p_body_leftcam; 0, 0, 0, 1];

% B. calculated by Direct_visual_lidar_calib 
% read Body_T_xt32
Body_T_xt32 = Body_T_Xt32();

% select xt32_t_Cam based on platform and compute initial Body_T_LeftCam ---
switch lower(platform)
    case 'suv'
        % SUV platform: fill in your real translation (m) and quaternion (qx,qy,qz,qw)
        tx = 0.0695283917427731;    % X offset of xt32 in camera frame
        ty = -0.008381612991474873;    % Y offset
        tz = -0.17223038663727022;    % Z offset
        qx = 0.019635536507920586;  % quaternion x
        qy = 0.7097335839994078;  % quaternion y
        qz = -0.7039714840647372;  % quaternion z
        qw = -0.017799861603211602; % quaternion w
    otherwise  
        % Other platforms: fill in your real translation & quaternion
        tx = 0.07493894778041606;
        ty = -0.16471796028449764;
        tz = -0.09812580091216104;
        qx = -0.007712187968660904;
        qy = -0.698602566500298;
        qz = 0.7154675436354508;
        qw = 0.0010817764035590007;
end

% build 4×4 Cam_T_xt32
Cam_T_xt32 = eye(4);
Cam_T_xt32(1:3,1:3) = quat2rotm([qw, qx, qy, qz]);  % [w x y z]
Cam_T_xt32(1:3,4)   = [tx; ty; tz];

% invert to init Body_T_Zed2LeftCam
T = Body_T_xt32 * inv(Cam_T_xt32);

end
