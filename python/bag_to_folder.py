"""
convert the rosbag file to a folder of files for each sensor

The rostopics

ros topics                                                  subfolder and filename
/x36d/imu   --> Imu                                         x36d/imu.txt;
/x36d/gnsspos --> NavSatFix                                 x36d/gnsspos.txt;
/x36d/inspos  --> NavSatFix                                 x36d/inspos.txt
/mti3dk/imu  --> Imu                                        mti3dk/imu.txt

/zed2i/zed_node/imu/data;                                   zed2i/imu.txt
/zed2i/zed_node/left_raw/image_raw_gray/compressed;         zed2i/left/times.txt zed2i/left/timestamps.jpg ...
/zed2i/zed_node/odom;                                       zed2i/odom.txt
/zed2i/zed_node/right_raw/image_raw_gray/compressed;        zed2i/right/times.txt zed2i/right/timestamps.jpg ...

/ars548;                                                    ars548/times.txt ars548/timestamps.pcd/bin ...

/radar_enhanced_pcl;                                        N/A
/radar_enhanced_pcl2                                        eagleg7/enhanced/times.txt eagleg7/enhanced/timestamps.pcd/bin ...
/radar_pcl;                                                 N/A
/radar_pcl2;                                                eagleg7/pcl/times.txt eagleg7/pcl/timestamps.pcd/bin ...
/radar_trk;                                                 eagleg7/trk/times.txt eagleg7/trk/timestamps.pcd/bin ...

/hesai/pandar                                               pandar/times.txt pandar/timestamps.pcd/bin ...

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

def msgfields_to_pcdfields(msgfields):
    """
    Convert a list of message fields to PCD fields.
    refer to https://docs.ros.org/en/noetic/api/sensor_msgs/html/msg/PointField.html
    and https://pointclouds.org/documentation/tutorials/pcd_file_format.html
    """
    fields = []
    sizes = []
    types = []
    counts = []
    # ros msg data types
    INT8    = 1
    UINT8   = 2
    INT16   = 3
    UINT16  = 4
    INT32   = 5
    UINT32  = 6
    FLOAT32 = 7
    FLOAT64 = 8
    for field in msgfields:
        fields.append(field.name)
        counts.append(1)
        if field.datatype == INT8:
            sizes.append(1)
            types.append('I')
        elif field.datatype == UINT8:
            sizes.append(1)
            types.append('U')
        elif field.datatype == INT16:
            sizes.append(2)
            types.append('I')
        elif field.datatype == UINT16:
            sizes.append(2)
            types.append('U')
        elif field.datatype == INT32:
            sizes.append(4)
            types.append('I')
        elif field.datatype == UINT32:
            sizes.append(4)
            types.append('U')
        elif field.datatype == FLOAT32:
            sizes.append(4)
            types.append('F')
        elif field.datatype == FLOAT64:
            sizes.append(8)
            types.append('F')
    return fields, sizes, types, counts


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
        '/hesai/pandar': ('xt32/timestamps.pcd', PointCloud2), # timestamps.pcd is in seconds, for example: 1716467261.084892076.pcd
    }

    PointCloud2fields = {
        '/ars548': 'x y z doppler intensity range_std azimuth_std elevation_std doppler_std',
        '/hesai/pandar': 'x y z intensity timestamp ring',
        '/radar_enhanced_pcl2': 'x y z Doppler Range Power Alpha Beta rangeAccu aziAccu eleAccu dopplerAccu recoveredSpeed dotFlags denoiseFlag historyFrameFlag dopplerCorrectionFlag',
        '/radar_pcl2': 'x y z Doppler Range Power Alpha Beta rangeAccu aziAccu eleAccu dopplerAccu recoveredSpeed dotFlags denoiseFlag historyFrameFlag dopplerCorrectionFlag',
    }

    hesai_fieldtypecodes = ('f', 'f', 'f', 'f', 'd', 'H')

    # check topics in bag file, if exist topic "/zed/*", if had, zedTopic_change = True
    zedTopic_change = False
    bagtopics_list = bag.get_type_and_topic_info().topics
    for topic in bagtopics_list:
        if '/zed/' in topic:
            zedTopic_change = True
            break

    bridge = CvBridge()

    for topic0, (subpath, msg_type) in topics.items():
        if topic0 not in bagtopics_list:
            continue
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
                                f'{msg.angular_velocity.x} {msg.angular_velocity.y} {msg.angular_velocity.z} '
                                f'{msg.orientation.x} {msg.orientation.y} {msg.orientation.z} {msg.orientation.w}\n')
                    elif msg_type == NavSatFix:
                        f.write(f'{msg.header.stamp.secs}.{msg.header.stamp.nsecs:09d} '
                                f'{msg.latitude:.9f} {msg.longitude:.9f} {msg.altitude:.3f} ')
                        f.write(f'{msg.position_covariance[0]} {msg.position_covariance[4]} {msg.position_covariance[8]} ')
                        f.write(f'{msg.status.status} {msg.status.service} {msg.position_covariance_type}\n')
                    elif msg_type == Odometry:
                        f.write(f'{msg.header.stamp.secs}.{msg.header.stamp.nsecs:09d} '
                                f'{msg.pose.pose.position.x} {msg.pose.pose.position.y} {msg.pose.pose.position.z} '
                                f'{msg.pose.pose.orientation.x} {msg.pose.pose.orientation.y} {msg.pose.pose.orientation.z} {msg.pose.pose.orientation.w}\n')   
        elif msg_type == PointCloud2:
            file_path = os.path.dirname(file_path)
            times_path = os.path.join(file_path, 'times.txt')
            fields = PointCloud2fields.get(topic, 'x y z').split()  # Use default fields xyz if not specified
            with open(times_path, 'w') as f1:
                for _, msg, _ in bag.read_messages(topics=[topic]):
                    timestamp = msg.header.stamp
                    fieldnames, sizes, types, counts = msgfields_to_pcdfields(msg.fields)
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
                            f.write(bytearray("SIZE 4 4 4 4 8 2\n", 'utf-8'))
                            f.write(bytearray("TYPE F F F F F U\n", 'utf-8'))
                            f.write(bytearray("COUNT 1 1 1 1 1 1\n", 'utf-8'))
                            f.write(bytearray(f"WIDTH {msg.width}\n", 'utf-8'))
                            f.write(bytearray("HEIGHT 1\n", 'utf-8'))
                            f.write(bytearray("VIEWPOINT 0 0 0 1 0 0 0\n", 'utf-8'))
                            f.write(bytearray(f"POINTS {msg.width}\n", 'utf-8'))
                            f.write(bytearray("DATA binary\n", 'utf-8'))
                            for point in pcl.read_points(msg, field_names=fields):
                                for i, p in enumerate(point):
                                    f.write(struct.pack(hesai_fieldtypecodes[i], p))
                        elif topic in ['/radar_enhanced_pcl2', '/radar_pcl2']:
                            f.write(bytearray("# .PCD v0.7 - Point Cloud Data file format\n", 'utf-8'))
                            f.write(bytearray("VERSION 0.7\n", 'utf-8'))
                            f.write(bytearray(f"FIELDS {' '.join(fieldnames)}\n", 'utf-8'))
                            f.write(bytearray(f"SIZE {' '.join(str(s) for s in sizes)}\n", 'utf-8'))
                            f.write(bytearray(f"TYPE {' '.join(types)}\n", 'utf-8'))
                            f.write(bytearray(f"COUNT {' '.join(str(c) for c in counts)}\n", 'utf-8'))
                            f.write(bytearray(f"WIDTH {msg.width}\n", 'utf-8'))
                            f.write(bytearray("HEIGHT 1\n", 'utf-8'))
                            f.write(bytearray("VIEWPOINT 0 0 0 1 0 0 0\n", 'utf-8'))
                            f.write(bytearray(f"POINTS {msg.width}\n", 'utf-8'))
                            f.write(bytearray("DATA binary\n", 'utf-8'))
                            for point in pcl.read_points(msg, field_names=fieldnames):
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
            times_path = os.path.join(file_path, 'times.txt')
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
                with open(times_path, 'a') as f:
                    f.write(f'{t.secs}.{t.nsecs:09d}\n')

    bag.close()
    get_frame_ids_from_bag(bag_file, output_dir)


def get_frame_ids_from_bag(bag_file, output_dir):
    frame_ids = {}
    with rosbag.Bag(bag_file, 'r') as bag:
        for topic, msg, t in bag.read_messages():
            # Assuming the message has a header with frame_id
            if hasattr(msg, 'header') and hasattr(msg.header, 'frame_id'):
                if topic not in frame_ids:
                    frame_ids[topic] = msg.header.frame_id
    with open(os.path.join(output_dir, 'frame_ids.txt'), 'w') as f:
        for topic, frame_id in frame_ids.items():
            f.write(f"{topic}: {frame_id}\n")
    return frame_ids

if __name__ == '__main__':
    import argparse
    parser = argparse.ArgumentParser(description='Convert ROS bag to a structured folder of files for each sensor.')
    parser.add_argument('bag_file', type=str, help='Input ROS bag file.')
    parser.add_argument('output_dir', type=str, help='Output directory for structured data.')
    args = parser.parse_args()

    extract_and_save(args.bag_file, args.output_dir)

