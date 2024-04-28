"""
convert a folder of snail radar format to a rosbag
"""

from sensor_msgs import point_cloud2
from sensor_msgs.msg import Imu, NavSatFix, CompressedImage, PointCloud2, PointCloud, ChannelFloat32
from geometry_msgs.msg import Point32
from nav_msgs.msg import Odometry
from sensor_msgs.point_cloud2 import PointField
import std_msgs.msg
import numpy as np
import struct
import os
import rosbag
import rospy

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

global data_type
data_type = 'binary'

def read_pcd_file(filename):
    """Read a PCD file and return a list of points with their attributes."""
    with open(filename, 'rb') as file:
        fields = []
        points = []
        while True:
            line = file.readline().decode('utf-8')
            if 'FIELDS' in line:
                fields = line.split()[1:]
            elif 'DATA' in line:
                data_type = line.split()[1]
                break
        
        while True:
            point_data = {}
            for field in fields:
                value_bytes = file.read(4)
                if not value_bytes:
                    break
                value = struct.unpack('f', value_bytes)[0]
                point_data[field] = value
            if not point_data:
                break
            points.append([point_data[field] for field in fields])

        return fields, points

def create_point_cloud2_msg(fields, points, timestamp):
    """Create a PointCloud2 message from fields and point data."""
    header = std_msgs.msg.Header()
    header.stamp = timestamp  # Adjust this according to your needs
    header.frame_id = "frame"  # Set the appropriate frame id

    topic_fields = []
    for i, field in enumerate(fields):
        topic_fields.append(PointField(field, 4*i, PointField.FLOAT32, 1))

    # Pass the point data directly
    cloud2_msg = point_cloud2.create_cloud(header, topic_fields, points)
    cloud2_msg.is_dense = True

    return cloud2_msg

def write_bag(input_folder, output_bag):
    bag = rosbag.Bag(output_bag, 'w')

    for topic, (subpath, msg_type) in topics.items():
        file_path = os.path.join(input_folder, subpath)

        if msg_type in [Imu, NavSatFix, Odometry]:
            with open(file_path, 'r') as f:
                for line in f:
                    parts = line.split()
                    msg = msg_type()
                    msg.header.stamp = rospy.Time(int(parts[0].split('.')[0]), int(parts[0].split('.')[1]))
                    if msg_type == Imu:
                        msg.linear_acceleration.x = float(parts[1])
                        msg.linear_acceleration.y = float(parts[2])
                        msg.linear_acceleration.z = float(parts[3])
                        msg.angular_velocity.x = float(parts[4])
                        msg.angular_velocity.y = float(parts[5])
                        msg.angular_velocity.z = float(parts[6])
                    elif msg_type == NavSatFix:
                        msg.latitude = float(parts[1])
                        msg.longitude = float(parts[2])
                        msg.altitude = float(parts[3])
                    elif msg_type == Odometry:
                        msg.pose.pose.position.x = float(parts[1])
                        msg.pose.pose.position.y = float(parts[2])
                        msg.pose.pose.position.z = float(parts[3])
                        msg.pose.pose.orientation.x = float(parts[4])
                        msg.pose.pose.orientation.y = float(parts[5])
                        msg.pose.pose.orientation.z = float(parts[6])
                        msg.pose.pose.orientation.w = float(parts[7])
                    bag.write(topic, msg, msg.header.stamp)
        elif msg_type == PointCloud2:
            file_path = os.path.dirname(file_path)
            for _, _, files in os.walk(file_path):
                for file in files:
                    if file.endswith('.pcd'):
                        fields, points = read_pcd_file(os.path.join(file_path, file))
                        timestamp = rospy.Time(int(file.split('.')[0]), int(file.split('.')[1]))
                        pc2 = create_point_cloud2_msg(fields, points, timestamp)
                        bag.write(topic, pc2, pc2.header.stamp)
        elif msg_type == CompressedImage:
            times_path = file_path
            with open(times_path, 'r') as f:
                for line in f:
                    timestamp = rospy.Time(float(line[:-1]))
                    image_path = file_path.replace('times.txt', f'{line[:-1]}.jpg')
                    with open(image_path, 'rb') as img:
                        msg = CompressedImage()
                        msg.header.stamp = timestamp
                        msg.format = 'jpeg'
                        msg.data = img.read()
                        bag.write(topic, msg, msg.header.stamp)
        elif msg_type == PointCloud:
            file_path = os.path.dirname(file_path)
            for _, _, files in os.walk(file_path):
                for file in files:
                    if file.endswith('.bin'):
                        with open(os.path.join(file_path, file), 'rb') as f:
                            # first 2 ii as points number and channels number
                            num_points = struct.unpack('I', f.read(4))[0]
                            num_channels = struct.unpack('I', f.read(4))[0]
                            # unpack 3d points
                            points = []
                            for i in range(num_points):
                                point = []
                                for j in range(3):
                                    point.append(struct.unpack('f', f.read(4))[0])
                                points.append(point)   
                            # unpack channels
                            channels = []
                            for i in range(num_channels):
                                channel = []
                                for j in range(num_points):
                                    channel.append(struct.unpack('f', f.read(4))[0])
                                channels.append(channel)
                            # create PointCloud message
                            msg = PointCloud()
                            msg.header.stamp = rospy.Time(int(file.split('.')[0]), int(file.split('.')[1]))
                            msg.header.frame_id = 'eagleg7'
                            for i in range(num_points):
                                point = Point32()
                                point.x = points[i][0]
                                point.y = points[i][1]
                                point.z = points[i][2]
                                msg.points.append(point)
                            for i in range(num_channels):
                                channel = ChannelFloat32()
                                channel.name = f'channel_{i}'
                                channel.values = channels[i]
                                msg.channels.append(channel)
                            bag.write(topic, msg, msg.header.stamp)     

    bag.close()

if __name__ == '__main__':
    import argparse
    parser = argparse.ArgumentParser(description='Convert a folder of snail radar format to a ROS bag.')
    parser.add_argument('input_folder', help='Path to the input folder.')
    parser.add_argument('output_bag', help='Path to the output bag file.')
    args = parser.parse_args()

    write_bag(args.input_folder, args.output_bag)
