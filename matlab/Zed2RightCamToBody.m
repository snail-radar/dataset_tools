function T = Zed2RightCamToBody()
T = Zed2LeftCamToBody() * Zed2RightCamToLeftCam();
end
