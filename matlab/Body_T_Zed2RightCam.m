function T = Body_T_Zed2RightCam(date)
% date is like 20231109 for Nov 9 2023.
T = Body_T_Zed2LeftCam(date) * Zed2LeftCam_T_Zed2RightCam();
end
