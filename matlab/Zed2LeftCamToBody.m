function T = Zed2LeftCamToBody()
% A. manually measured
R_body_leftcam = [0, 0, 1; -1, 0, 0; 0, -1, 0];
p_body_leftcam = [0.125; 0.07; -0.195];
T = [R_body_leftcam p_body_leftcam; 0, 0, 0, 1];

% B. calculated from /zed2i/zed_node/left_cam_imu_transform
% zed2leftcamx the left camera frame with forward left up orientation
% zed2imu the imu frame with forward right down orientation
tx = -0.0020000000949949026;
ty = -0.023000003769993782;
tz = 0.0002200000308221206;
qx = -0.0008717564051039517;
qy = -0.00139715860132128;
qz = -0.0010711626382544637;
qw = 0.9999980330467224;
q_zed2leftcamx_zed2imu = [qw, qx, qy, qz];
T_zed2leftcamx_zed2imu = eye(4);
T_zed2leftcamx_zed2imu(1:3, 1:3) = quat2dcm(q_zed2leftcamx_zed2imu);
T_zed2leftcamx_zed2imu(1:3, 4) = [tx; ty; tz];

T_rdf_zed2leftcamx = eye(4);
T_rdf_zed2leftcamx(1:3, 1:3) = [0, -1, 0; 0, 0, -1; 1, 0, 0];
T_zed2leftcam_imu = T_rdf_zed2leftcamx * T_zed2leftcamx_zed2imu;
T = Zed2ImuToBody() * inv(T_zed2leftcam_imu);
end
