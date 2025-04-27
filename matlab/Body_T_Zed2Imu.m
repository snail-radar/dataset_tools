function T = Body_T_Zed2Imu(date)
%% zed2 IMU frame to body frame transformation that 
% transforms a point in zed2 IMU frame to the body frame.
R_body_zed2 = eye(3);
p_body_zed2 = [0.125; 0.0; -0.195];
T = [R_body_zed2 p_body_zed2; 0, 0, 0, 1]; % manually measured value

% estimated by lidar IMU init. This value is suitable for ebike and
% handheld seqs but not SUV seqs.
T_Zed2Imu_Xt32 = ...
    [-0.020255 -0.999744  0.010133 -0.100633; ...
    0.999784 -0.020206  0.004917 -0.030823; ...
    -0.004711  0.010231  0.999937  0.197481; ...
    0.000000  0.000000  0.000000  1.000000];

T_Zed2Imu_Zed2LeftCam = Zed2Imu_T_Zed2LeftCam();
T = Body_T_Zed2LeftCam(date) * inv(T_Zed2Imu_Zed2LeftCam);
end
