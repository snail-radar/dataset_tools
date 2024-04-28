function T = Zed2LeftCam_T_Zed2RightCam()
R_leftcam_rightcam = eye(3);
p_leftcam_rightcam = [0.12; 0; 0];
T = [R_leftcam_rightcam p_leftcam_rightcam; 0, 0, 0, 1];
end
