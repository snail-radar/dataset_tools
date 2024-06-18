function T = Zed2Imu_T_Zed2LeftCam()
Pq = [-0.0020000000949949026, -0.023000003769993782, 0.0002200000308221206, ...
      -0.0008717564051039517, -0.00139715860132128, -0.0010711626382544637, 0.9999980330467224];
% left_cam_imu_transform message published by zed uses the forward left up frame for
% both camera and IMU.
Flu_R_Rdf = [0, 0, 1; -1, 0, 0; 0, -1, 0]; % we use the right down forward frame for the camera.
Flu_T_Rdf = [Flu_R_Rdf, [0; 0; 0]; 0, 0, 0, 1];
camFlu_T_imu = T_from_Pq(Pq);
imu_T_camRdf = camFlu_T_imu \ Flu_T_Rdf;
T = imu_T_camRdf;
end
