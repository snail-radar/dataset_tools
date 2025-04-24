function T = Body_T_Zed2LeftCam(date)
% date is like 20231109 for Nov 9 2023.
% A. manually measured
R_body_leftcam = [0, 0, 1; -1, 0, 0; 0, -1, 0];
p_body_leftcam = [0.125; 0.07; -0.195];
T = [R_body_leftcam p_body_leftcam; 0, 0, 0, 1];

% B. calculated by Direct_visual_lidar_calib 
Body_T_xt32 = Body_T_Xt32();

% Apr 22 data - calibrated by "Direct_visual_lidar_calib"
% the results obtained for the current rig whose extrinsics are expected to be very close to that of the SUV seqs as they were collected at last. 
% Moreover, note that the SUV extrinsics are obtained by direct visual lidar calib using multiple static subseqs \\
% and the ebike extrinsics are obtained by dvlc using a static ebike subseq.
tx = 0.16766995495366466;
ty = 0.1323746871838674;
tz = -0.1604850280896764;
qx = 0.018921549882585903;
qy = 0.7149261548397216;
qz = -0.6986643622889027;
qw = -0.019765549412358693;

if date > 20231200 % SUV platform
    tx = 0.0695283917427731; 
    ty = -0.008381612991474873;
    tz = -0.17223038663727022;
    qx = 0.019635536507920586;
    qy = 0.7097335839994078;
    qz = -0.7039714840647372;
    qw = -0.017799861603211602;
else % handheld or ebike
    tx = 0.07493894778041606;
    ty = -0.16471796028449764;
    tz = -0.09812580091216104;
    qx = -0.007712187968660904;
    qy = -0.698602566500298;
    qz = 0.7154675436354508;
    qw = 0.0010817764035590007;
end

xt32_T_leftcam = eye(4);
xt32_T_leftcam(1:3,1:3) = quat2rotm([qw, qx, qy, qz]);

% use manually measured translation
xt32_T_leftcam(1:3, 4) = Body_T_xt32(1:3, 1:3)' * p_body_leftcam;

T = Body_T_xt32 * xt32_T_leftcam;
end
