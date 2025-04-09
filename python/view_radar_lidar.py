
#!/usr/bin/env python

# Give a ros1 bag of ars radar PointCloud2 topic /ars548
# and oculii radar PointCloud2 topic /radar_pcl2, and pandar xt32 lidar PointCloud2 topic /hesai/pandar
# also their relative poses ars_T_xt32, oculii_T_xt32
# Display these point clouds in rviz by publishing these point clouds and their static transforms.

# To use it in a ros1 environment,
# 1. open a terminal, play bag
# python3 view_radar_lidar.py /media/jhuai/My_Book1/jhuai/data/zip/20231105/data6.bag
# 2. open another terminal, run rviz
# rviz -d view_radar_lidar.rviz

import rospy
import rosbag
from sensor_msgs.msg import PointCloud2
from geometry_msgs.msg import TransformStamped
import tf2_ros
import tf
import numpy as np

def matrix_to_transform(matrix, parent_frame, child_frame, stamp):
    """
    Converts a 4x4 homogeneous transformation matrix to a TransformStamped message.
    """
    # Extract translation from the last column
    translation = matrix[0:3, 3]
    # Compute quaternion from rotation matrix
    quat = tf.transformations.quaternion_from_matrix(matrix)
    
    t = TransformStamped()
    t.header.stamp = stamp
    t.header.frame_id = parent_frame
    t.child_frame_id = child_frame
    t.transform.translation.x = translation[0]
    t.transform.translation.y = translation[1]
    t.transform.translation.z = translation[2]
    t.transform.rotation.x = quat[0]
    t.transform.rotation.y = quat[1]
    t.transform.rotation.z = quat[2]
    t.transform.rotation.w = quat[3]
    return t

def publish_static_transforms():
    """
    Publishes static transforms for the sensors relative to the lidar frame.
    Adjust the transformation matrices with your actual calibration data.
    """
    static_broadcaster = tf2_ros.StaticTransformBroadcaster()
    xt32_T_ars = np.array([
        [-0.0148246945939438,  0.999869704931098,  -0.00638767491813221, 0],
        [-0.967459492843472,   -0.0127297492912655,  0.252705525895304,   0],
        [ 0.252591285490489,    0.00992609917652017,  0.967522151941183,   0.07],
        [ 0,                   0,                    0,                  1]
    ])

    xt32_T_oculii = np.array([
        [-0.0197501325711932,  0.999429818150974,   0.0273856103418523,  0],
        [-0.999756831101593,   -0.0200104940487212,  0.00926598195627003, -0.07],
        [ 0.0098086977351427,  -0.027195946430554,   0.999581997333877,  -0.115],
        [ 0,                   0,                    0,                  1]
    ])
    now = rospy.Time.now()
    # Create TransformStamped messages:
    t_ars = matrix_to_transform(xt32_T_ars, parent_frame="PandarXT-32", child_frame="ars548", stamp=now)
    t_oculii = matrix_to_transform(xt32_T_oculii, parent_frame="PandarXT-32", child_frame="oculii", stamp=now)

    # Broadcast the static transforms
    static_broadcaster.sendTransform([t_ars, t_oculii])
    rospy.loginfo("Published static transforms for 'ars' and 'oculii' relative to 'xt32'.")

def main():
    import argparse
    parser = argparse.ArgumentParser(description='Publish PointCloud2 data from a rosbag file')
    parser.add_argument('bag_file', type=str, help='Path to the rosbag file. '
    			'Be sure to adjust the static transform matrices in publish_static_transforms() using the matlab calibration results')
    args = parser.parse_args()

    rospy.init_node('pointcloud_publisher', anonymous=True)
    
    # Create publishers for the PointCloud2 topics
    pub_ars = rospy.Publisher('/ars548', PointCloud2, queue_size=10)
    pub_oculii = rospy.Publisher('/radar_pcl2', PointCloud2, queue_size=10)
    pub_lidar = rospy.Publisher('/hesai/pandar', PointCloud2, queue_size=10)
    
    # Publish static transforms once (they are latched)
    publish_static_transforms()
    
    rospy.loginfo("Opening bag file: {}".format(args.bag_file))
    
    bag = rosbag.Bag(args.bag_file, 'r')

    rate = rospy.Rate(10)  # Publishing rate in Hz
    for topic, msg, t in bag.read_messages():
        if topic == '/ars548':
            pub_ars.publish(msg)
        elif topic == '/radar_pcl2':
            pub_oculii.publish(msg)
        elif topic == '/hesai/pandar':
            pub_lidar.publish(msg)
        rate.sleep()
    
    bag.close()

if __name__ == '__main__':
    main()
