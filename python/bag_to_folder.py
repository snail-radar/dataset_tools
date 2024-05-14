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
import rosbag
from sensor_msgs.msg import Imu, NavSatFix, CompressedImage, PointCloud2, PointCloud
from nav_msgs.msg import Odometry
import sensor_msgs.point_cloud2 as pcl
from cv_bridge import CvBridge
import cv2
import struct

def extract_and_save(bag_file, output_dir):
    bag = rosbag.Bag(bag_file, 'r')
    topics = {
        '/x36d/imu_raw': ('x36d/imu.txt', Imu),
        '/x36d/gnss': ('x36d/gnss.txt', NavSatFix),
        '/x36d/gnss_ins': ('x36d/gnss_ins.txt', NavSatFix),
        '/mti3dk/imu': ('mti3dk/imu.txt', Imu),
        '/zed2i/zed_node/imu/data': ('zed2i/imu.txt', Imu),
        '/zed2i/zed_node/odom': ('zed2i/odom.txt', Odometry),
        '/zed2i/zed_node/left_raw/image_raw_gray/compressed': ('zed2i/left/times.txt', CompressedImage),
        '/zed2i/zed_node/right_raw/image_raw_gray/compressed': ('zed2i/right/times.txt', CompressedImage),
        '/ars548': ('ars548/points/timestamps.pcd', PointCloud2),
        '/radar_enhanced_pcl2': ('eagleg7/enhanced/timestamps.pcd', PointCloud2),
        '/radar_pcl2': ('eagleg7/pcl/timestamps.pcd', PointCloud2),
        '/radar_trk': ('eagleg7/trk/timestamps.pcd', PointCloud),
        '/hesai/pandar': ('xt32/timestamps.pcd', PointCloud2),
    }

    PointCloud2fields = {
        '/ars548': 'x y z doppler intensity range_std azimuth_std elevation_std doppler_std',
        '/hesai/pandar': 'x y z intensity timestamp ring',
        '/radar_enhanced_pcl2': 'x y z Doppler Range Power Alpha Beta rangeAccu aziAccu eleAccu dopplerAccu recoveredSpeed dotFlags denoiseFlag historyFrameFlag dopplerCorrectionFlag',
        '/radar_pcl2': 'x y z Doppler Range Power Alpha Beta rangeAccu aziAccu eleAccu dopplerAccu recoveredSpeed dotFlags denoiseFlag historyFrameFlag dopplerCorrectionFlag',
    }

    # check topics in bag file, if exist topic "/zed/*", if had, zedTopic_change = True
    zedTopic_change = False
    bagtopics_list = bag.get_type_and_topic_info().topics
    for topic in bagtopics_list:
        if '/zed/' in topic:
            zedTopic_change = True
            break


    bridge = CvBridge()

    for topic0, (subpath, msg_type) in topics.items():
        file_path = os.path.join(output_dir, subpath)
        os.makedirs(os.path.dirname(file_path), exist_ok=True)
        topic = topic0
        if zedTopic_change and '/zed/' in topic0:
            topic = topic.replace('/zed/', '/zed2i/')

        if msg_type in [Imu, NavSatFix, Odometry]:
            with open(file_path, 'w') as f:
                for _, msg, _ in bag.read_messages(topics=[topic]):
                    if msg_type == Imu:
                        f.write(f'{msg.header.stamp.secs}.{msg.header.stamp.nsecs:09d} '
                                f'{msg.linear_acceleration.x} {msg.linear_acceleration.y} {msg.linear_acceleration.z} '
                                f'{msg.angular_velocity.x} {msg.angular_velocity.y} {msg.angular_velocity.z}\n')
                    elif msg_type == NavSatFix:
                        f.write(f'{msg.header.stamp.secs}.{msg.header.stamp.nsecs:09d} '
                                f'{msg.latitude} {msg.longitude} {msg.altitude} ')
                        f.write(f'{msg.position_covariance[0]} {msg.position_covariance[4]} {msg.position_covariance[8]}\n')
                    elif msg_type == Odometry:
                        f.write(f'{msg.header.stamp.secs}.{msg.header.stamp.nsecs:09d} '
                                f'{msg.pose.pose.position.x} {msg.pose.pose.position.y} {msg.pose.pose.position.z} '
                                f'{msg.pose.pose.orientation.x} {msg.pose.pose.orientation.y} {msg.pose.pose.orientation.z} {msg.pose.pose.orientation.w}\n')   
        elif msg_type == PointCloud2:
            file_path = os.path.dirname(file_path)
            times_path = os.path.join(file_path, 'timestamps.txt')
            fields = PointCloud2fields.get(topic, 'x y z').split()  # Use default fields if not specified
            with open(times_path, 'w') as f1:
                for _, msg, _ in bag.read_messages(topics=[topic]):
                    timestamp = msg.header.stamp
                    f1.write(f'{timestamp.secs}.{timestamp.nsecs:09d}\n')
                    pcd_path = os.path.join(file_path, f'{timestamp.secs}.{timestamp.nsecs:09d}.pcd')
                    with open(pcd_path, 'wb') as f:
                        if topic in ['/ars548']:
                            f.write(bytearray("# .PCD v0.7 - Point Cloud Data file format\n", 'utf-8'))
                            f.write(bytearray("VERSION 0.7\n", 'utf-8'))
                            f.write(bytearray(f"FIELDS {' '.join(fields)}\n", 'utf-8'))
                            f.write(bytearray("SIZE 4 4 4 4 4 4 4 4 4\n", 'utf-8'))
                            f.write(bytearray("TYPE F F F F F F F F F\n", 'utf-8'))
                            f.write(bytearray("COUNT 1 1 1 1 1 1 1 1 1\n", 'utf-8'))
                            f.write(bytearray(f"WIDTH {msg.width}\n", 'utf-8'))
                            f.write(bytearray("HEIGHT 1\n", 'utf-8'))
                            f.write(bytearray("VIEWPOINT 0 0 0 1 0 0 0\n", 'utf-8'))
                            f.write(bytearray(f"POINTS {msg.width}\n", 'utf-8'))
                            f.write(bytearray("DATA binary\n", 'utf-8'))
                            for point in pcl.read_points(msg, field_names=fields):
                                for p in point:
                                    f.write(struct.pack('f', p))
                        elif topic in ['/hesai/pandar']:
                            f.write(bytearray("# .PCD v0.7 - Point Cloud Data file format\n", 'utf-8'))
                            f.write(bytearray("VERSION 0.7\n", 'utf-8'))
                            f.write(bytearray(f"FIELDS {' '.join(fields)}\n", 'utf-8'))
                            f.write(bytearray("SIZE 4 4 4 4 4 4\n", 'utf-8'))
                            f.write(bytearray("TYPE F F F F F F\n", 'utf-8'))
                            f.write(bytearray("COUNT 1 1 1 1 1 1\n", 'utf-8'))
                            f.write(bytearray(f"WIDTH {msg.width}\n", 'utf-8'))
                            f.write(bytearray("HEIGHT 1\n", 'utf-8'))
                            f.write(bytearray("VIEWPOINT 0 0 0 1 0 0 0\n", 'utf-8'))
                            f.write(bytearray(f"POINTS {msg.width}\n", 'utf-8'))
                            f.write(bytearray("DATA binary\n", 'utf-8'))
                            for point in pcl.read_points(msg, field_names=fields):
                                for p in point:
                                    f.write(struct.pack('f', p))
                        elif topic in ['/radar_enhanced_pcl2', '/radar_pcl2']:
                            f.write(bytearray("# .PCD v0.7 - Point Cloud Data file format\n", 'utf-8'))
                            f.write(bytearray("VERSION 0.7\n", 'utf-8'))
                            f.write(bytearray(f"FIELDS {' '.join(fields)}\n", 'utf-8'))
                            f.write(bytearray("SIZE 4 4 4 4 4 4 4 4 4 4 4 4 4 4 4 4 4\n", 'utf-8'))
                            f.write(bytearray("TYPE F F F F F F F F F F F F F F F F F\n", 'utf-8'))
                            f.write(bytearray("COUNT 1 1 1 1 1 1 1 1 1 1 1 1 1 1 1 1 1\n", 'utf-8'))
                            f.write(bytearray(f"WIDTH {msg.width}\n", 'utf-8'))
                            f.write(bytearray("HEIGHT 1\n", 'utf-8'))
                            f.write(bytearray("VIEWPOINT 0 0 0 1 0 0 0\n", 'utf-8'))
                            f.write(bytearray(f"POINTS {msg.width}\n", 'utf-8'))
                            f.write(bytearray("DATA binary\n", 'utf-8'))
                            for point in pcl.read_points(msg, field_names=fields):
                                for p in point:
                                    f.write(struct.pack('f', p))
        elif msg_type == CompressedImage:
            times_path = file_path
            file_path = os.path.dirname(file_path)
            idx = 0
            for _, msg, t in bag.read_messages(topics=[topic]):
                cv_image = bridge.compressed_imgmsg_to_cv2(msg)
                image_path = os.path.join(file_path, f'{t.secs}.{t.nsecs:09d}.jpg')
                cv2.imwrite(image_path, cv_image)
                with open(times_path, 'a') as f:
                    f.write(f'{t.secs}.{t.nsecs:09d}\n')
                idx += 1
        elif msg_type == PointCloud:
            # Save PointCloud data directly as binary
            file_path = os.path.dirname(file_path)
            for _, msg, t in bag.read_messages(topics=[topic]):
                pcd_path = os.path.join(file_path, f'{t.secs}.{t.nsecs:09d}.bin')
                with open(pcd_path, 'wb') as f:
                    f.write(struct.pack('ii', len(msg.points), len(msg.channels)))
                    for point in msg.points:
                        f.write(struct.pack('fff', point.x, point.y, point.z))
                    for channel in msg.channels:
                        # print('channels: ', channel.name, len(channel.values))
                        for data in channel.values:
                            f.write(struct.pack('f', data))
                    

    bag.close()

if __name__ == '__main__':
    import argparse
    parser = argparse.ArgumentParser(description='Convert ROS bag to a structured folder of files for each sensor.')
    parser.add_argument('bag_file', type=str, help='Input ROS bag file.')
    parser.add_argument('output_dir', type=str, help='Output directory for structured data.')
    args = parser.parse_args()

    extract_and_save(args.bag_file, args.output_dir)
    # zip
    os.system(f'zip -r {args.output_dir}.zip {args.output_dir}')


