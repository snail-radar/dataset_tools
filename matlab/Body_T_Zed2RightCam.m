function T = Body_T_Zed2RightCam()
T = Body_T_Zed2LeftCam() * Zed2LeftCam_T_Zed2RightCam();
end
