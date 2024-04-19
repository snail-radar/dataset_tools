"""
convert the rosbag file to a folder of files for each sensor

The rostopics

ros topics                                                  subfolder and filename
/x36d/imu   --> Imu                                         x36d/imu.txt;
/x36d/gnsspos --> NavSatFix                                 x36d/gnsspos.txt;
/x36d/inspos  --> NavSatFix                                 x36d/inspos.txt
/mti3dk/imu  --> Imu                                        mti3dk/imu.txt

/zed2i/zed_node/imu/data;                                   zed2i/imu.txt
/zed2i/zed_node/left_raw/image_raw_gray/compressed;         zed2i/left/timestamps.txt zed2i/left/timestampinnanosecs.jpg ...
/zed2i/zed_node/odom;                                       zed2i/odom.txt
/zed2i/zed_node/right_raw/image_raw_gray/compressed;        zed2i/right/timestamps.txt zed2i/right/timestampinnanosecs.jpg ...

/ars548;                                                    ars548/timestamps.txt ars548/timestampinnanosecs.pcd/bin ...

/radar_enhanced_pcl;                                        N/A
/radar_enhanced_pcl2                                        eagleg7/enhanced/timestamps.txt eagleg7/enhanced/timestampinnanosecs.pcd/bin ...
/radar_pcl;                                                 N/A
/radar_pcl2;                                                eagleg7/pcl/timestamps.txt eagleg7/pcl/timestampinnanosecs.pcd/bin ...
/radar_trk;                                                 eagleg7/trk/timestamps.txt eagleg7/trk/timestampinnanosecs.pcd/bin ...

/hesai/pandar                                               pandar/timestamps.txt pandar/timestampinnanosecs.pcd/bin ...

"""

import os
import sys

# refer to [coloradar tools](https://github.com/JzHuai0108/coloradar_tools/blob/master/python/bag_to_dataset_fmt.py)

