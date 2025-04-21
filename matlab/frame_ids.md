| topic                                | type                              | frame_id         | child_frame_id     | comment                                          |
|--------------------------------------|-----------------------------------|------------------|--------------------|--------------------------------------------------|
| /ars548                              | sensor_msgs/PointCloud2           | ars548           | -                  | -                                                |
| /hesai/pandar                        | sensor_msgs/PointCloud2           | PandarXT-32      | -                  | -                                                |
| /mti3dk/imu                          | sensor_msgs/Imu                   | mti3dk           | -                  | -                                                |
| /radar_enhanced_pcl2                 | sensor_msgs/PointCloud2           | oculii           | -                  | "x y z in oculii frame, alpha beta in oculii_native frame" |
| /radar_pcl2                          | sensor_msgs/PointCloud2           | oculii           | -                  | "x y z in oculii frame, alpha beta in oculii_native frame" |
| /radar_trk                           | sensor_msgs/PointCloud            | oculii           | -                  | "x y z in oculii frame, alpha beta in oculii_native frame" |
| /x36d/gnss                           | sensor_msgs/NavSatFix             | x36d             | -                  | -                                                |
| /x36d/gnss_ins                       | sensor_msgs/NavSatFix             | x36d             | -                  | -                                                |
| /x36d/imu_raw                        | sensor_msgs/Imu                   | x36d             | -                  | -                                                |
| /zed2i/zed_node/imu/data             | sensor_msgs/Imu                   | zed2i_imu_link   | -                  | -                                                |
| /zed2i/zed_node/left_cam_imu_transform | geometry_msgs/Transform         | -                | -                  | -                                                |
| /zed2i/zed_node/left_raw/camera_info | sensor_msgs/CameraInfo            | -                | -                  | -                                                |
| /zed2i/zed_node/left_raw/image_raw_gray/compressed | sensor_msgs/CompressedImage | -          | -                  | -                                                |
| /zed2i/zed_node/odom                 | nav_msgs/Odometry                 | zed2i_odom       | zed2i_base_link    | -                                                |
| /zed2i/zed_node/right_raw/camera_info | sensor_msgs/CameraInfo           | -                | -                  | -                                                |
| /zed2i/zed_node/right_raw/image_raw_gray/compressed | sensor_msgs/CompressedImage | -        | -                  | -                                                |
