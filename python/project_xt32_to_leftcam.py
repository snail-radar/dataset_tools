#!/usr/bin/env python3
import numpy as np
import rosbag
from PIL import Image, ImageDraw
import sensor_msgs.point_cloud2 as pc2
from scipy.spatial.transform import Rotation
import cv2
import matplotlib.pyplot as plt
import rospy
import argparse

class BagProjector:
    def __init__(self, bag_path):
        self.bag = rosbag.Bag(bag_path)

    def quaternion_to_matrix(self, xyzqxqyqzqw):
        """
        Convert xyz + quaternion to a 4x4 transformation matrix.
        :param xyzqxqyqzqw: List containing translation and quaternion
        :return: 4x4 transformation matrix
        """
        translation = np.array(xyzqxqyqzqw[:3])
        quaternion = xyzqxqyqzqw[3:]
        rot = Rotation.from_quat(quaternion).as_matrix()

        T = np.eye(4)
        T[:3, :3] = rot
        T[:3, 3] = translation
        return T

    def extract_data(self, lidar_topic, image_topic, max_duration=1.0):
        """
        Extract synchronized LiDAR and image data from the bag.
        :param lidar_topic: LiDAR topic name
        :param image_topic: Image topic name
        :param max_duration: Maximum duration for synchronization
        :return: First LiDAR and image messages
        """
        start_time = None
        for topic, msg, t in self.bag.read_messages(topics=[lidar_topic, image_topic]):
            start_time = t.to_sec()
            break

        lidar_msgs = []
        image_msgs = []
        for topic, msg, t in self.bag.read_messages(
                topics=[lidar_topic, image_topic],
                start_time=rospy.Time.from_sec(start_time),
                end_time=rospy.Time.from_sec(start_time + max_duration)):

            if topic == lidar_topic:
                lidar_msgs.append(msg)
            elif topic == image_topic:
                image_msgs.append(msg)

        return lidar_msgs[0], image_msgs[0]

    def ros_img_to_numpy(self, img_msg, image_topic):
        """
        Convert ROS image message to numpy array.
        :param img_msg: ROS image message
        :param image_topic: Image topic name
        :return: Numpy array of the image
        """
        if 'compressed' in image_topic:
            image = np.frombuffer(img_msg.data, dtype=np.uint8)
            image = cv2.imdecode(image, cv2.IMREAD_COLOR)
        else:
            image_data = np.frombuffer(img_msg.data, dtype=np.uint8).reshape(img_msg.height, img_msg.width, 4)
            image = cv2.cvtColor(image_data, cv2.COLOR_BGRA2BGR)
        return image

    def project_lidar_to_image(self, pc_msg, img_msg, T_cam_lidar, K, image_topic):
        """
        Project LiDAR points onto the image.
        :param pc_msg: LiDAR point cloud message
        :param img_msg: Image message
        :param T_cam_lidar: Transformation matrix from LiDAR to camera
        :param K: Camera intrinsic matrix
        :param image_topic: Image topic name
        :return: Projected points, 3D points, and image
        """
        points = np.array(list(pc2.read_points(pc_msg, field_names=("x", "y", "z"), skip_nans=True)))
        img = self.ros_img_to_numpy(img_msg, image_topic)
        height, width = img.shape[:2]

        points_homo = np.column_stack([points, np.ones(len(points))])
        points_cam = (T_cam_lidar @ points_homo.T).T[:, :3]

        valid = points_cam[:, 2] > 0
        points_cam = points_cam[valid]

        projected = (K @ points_cam.T).T
        projected = (projected[:, :2] / projected[:, 2:]).astype(int)

        in_frame = (projected[:, 0] >= 0) & (projected[:, 0] < width) & \
                   (projected[:, 1] >= 0) & (projected[:, 1] < height)

        return projected[in_frame], points_cam[in_frame], img

    def visualize(self, projected_points, points_3d, image):
        """
        Visualize the projection result.
        :param projected_points: Projected 2D points
        :param points_3d: 3D points in camera frame
        :param image: Image to overlay the points
        """
        depths = points_3d[:, 2]
        norm_depths = (depths - depths.min()) / (depths.max() - depths.min())
        colors = (plt.cm.turbo(norm_depths)[:, :3] * 255).astype(int)

        img_pil = Image.fromarray(image)
        draw = ImageDraw.Draw(img_pil)

        for pt, color in zip(projected_points, colors):
            draw.ellipse([(pt[0] - 1, pt[1] - 1), (pt[0] + 1, pt[1] + 1)], fill=tuple(color))

        img_pil.show()


if __name__ == "__main__":
    # Parse input arguments
    parser = argparse.ArgumentParser(description="LiDAR to Camera Projection")
    parser.add_argument("--bag_path", type=str, required=True, help="Path to the ROS bag file")
    parser.add_argument("--lidar_topic", type=str, required=True, help="LiDAR topic name")
    parser.add_argument("--image_topic", type=str, required=True, help="Image topic name")
    parser.add_argument("--date_input", type=str, required=True, help="Date input in YYYYMMDD format")
    args = parser.parse_args()

    # Split date for configuration selection
    SPLIT_DATE = 20231207

    if int(args.date_input) > 20231207:  # SUV platform
        extrinsics = [0.0695283917427731, -0.008381612991474873, -0.17223038663727022,
                       0.019635536507920586, 0.7097335839994078, -0.7039714840647372, -0.017799861603211602]
        intrinsics = np.array([[266.52, 0, 345.01],
                                [0, 266.8025, 189.99125],
                                [0, 0, 1]])
    else:  # Handheld/ebike platform
        extrinsics = [0.16766995495366466, 0.1323746871838674, -0.1604850280896764,
                       0.018921549882585903, 0.7149261548397216, -0.6986643622889027, -0.019765549412358693]
        intrinsics = np.array([
                        [533.0399780273438, 0, 659.02001953125],
                        [0, 533.60498046875, 364.9825134277344],
                        [0, 0, 1]])

    # Initialize the projector
    projector = BagProjector(args.bag_path)
    T_lidar_cam = projector.quaternion_to_matrix(extrinsics)
    T_cam_lidar = np.linalg.inv(T_lidar_cam)

    # Extract data and project LiDAR points onto the image
    pc_msg, img_msg = projector.extract_data(args.lidar_topic, args.image_topic)
    projected_pts, pts_3d, img = projector.project_lidar_to_image(pc_msg, img_msg, T_cam_lidar, intrinsics, args.image_topic)

    # Visualize the result
    projector.visualize(projected_pts, pts_3d, img)