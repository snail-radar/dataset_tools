#!/usr/bin/env python3
import numpy as np
import rosbag
import sensor_msgs.point_cloud2 as pc2
from scipy.spatial.transform import Rotation
import cv2
import argparse
from scipy.spatial.transform import Slerp
import matplotlib.pyplot as plt
import sys
import os
from collections import deque

class BagProjector:
    def __init__(self, bag_path, gt_pose_file):
        # Load ROS bag and distortion coefficients
        self.bag = rosbag.Bag(bag_path)
        # Load ground truth poses from CSV: time,x,y,z,qx,qy,qz,qw
        data = np.loadtxt(gt_pose_file, delimiter=',')
        # Separate times and pose arrays for fast interpolation
        self.times = data[:, 0]
        self.poses = data[:, 1:8]  # [x,y,z,qx,qy,qz,qw]

    def pq_to_mat(self, xyzqxqyqzqw): # q : xyzw
        # Convert translation+quaternion to 4x4 transform
        arr = np.asarray(xyzqxqyqzqw, dtype=float).flatten()
        # Expect at least 7 elements: x,y,z,qx,qy,qz,qw
        if arr.size < 7:
            raise ValueError(f"Pose array too short, expected >=7 elements, got {arr.size}")
        # Translation and quaternion (only first four quaternion components)
        trans = arr[0:3]
        quat = arr[3:7]
        # Build rotation matrix
        R = Rotation.from_quat(quat).as_matrix()
        T = np.eye(4)
        T[:3, :3] = R
        T[:3, 3] = trans
        return T

    def interp_mat(self, t):
        if t < self.times[0] or t > self.times[-1]:
            return None

        # find the time interval
        idx = np.searchsorted(self.times, t)
        t1, t2 = self.times[idx - 1], self.times[idx]
        alpha = (t - t1) / (t2 - t1)

        # extract translation and quaternion
        trans1, trans2 = self.poses[idx - 1, :3], self.poses[idx, :3]
        quat1, quat2 = self.poses[idx - 1, 3:], self.poses[idx, 3:]

        # linear interpolate translation
        interpolated_translation = (1 - alpha) * trans1 + alpha * trans2

        # use slerp to interpolate quaternion
        rotations = Rotation.from_quat([quat1, quat2])
        slerp = Slerp([t1, t2], rotations)
        interpolated_rotation = slerp(t).as_matrix()

        T = np.eye(4)
        T[:3, :3] = interpolated_rotation
        T[:3, 3] = interpolated_translation
        return T

    def img_msg_to_numpy(self, img_msg, topic):
        # Convert ROS Image or CompressedImage to BGR numpy array
        if 'compressed' in topic:
            arr = np.frombuffer(img_msg.data, np.uint8)
            return cv2.imdecode(arr, cv2.IMREAD_COLOR)
        raw = np.frombuffer(img_msg.data, np.uint8).reshape(img_msg.height, img_msg.width, 4)
        return cv2.cvtColor(raw, cv2.COLOR_BGRA2BGR)

    def project(self, pc_msg, img, T_cam_l, K, dist_coeffs):
        # Project point cloud into the image
        points = np.array(list(pc2.read_points(pc_msg, field_names=('x','y','z'), skip_nans=True)))
        # Transform to camera frame and filter
        pts_cam = (T_cam_l @ np.hstack([points, np.ones((len(points),1))]).T).T[:, :3]
        valid_cam = pts_cam[:, 2] > 0
        pts_cam = pts_cam[valid_cam]
        # Project with distortion
        pts2d, _ = cv2.projectPoints(pts_cam, np.zeros(3), np.zeros(3), K, dist_coeffs)
        pts2d = pts2d.reshape(-1, 2).astype(int)
        h, w = img.shape[:2]
        valid2d = (pts2d[:,0]>=0)&(pts2d[:,0]<w)&(pts2d[:,1]>=0)&(pts2d[:,1]<h)
        return pts2d[valid2d], pts_cam[valid2d]

    def visualize(self, img, pts2d, pts_cam):
        if pts_cam.ndim != 2 or pts_cam.shape[1] < 3:
            raise ValueError("`pts_cam` must be a 2D array with at least 3 columns (x, y, z).")
        vis = img.copy()
        depths = pts_cam[:, 2]
        nd = (depths - depths.min()) / (depths.max() - depths.min())
        colors = (plt.cm.turbo(nd)[:, :3] * 255).astype(int)
        for (x, y), c in zip(pts2d, colors):
            cv2.circle(vis, (int(x), int(y)), 1, (int(c[2]), int(c[1]), int(c[0])), 1)
        win = 'Projection'
        cv2.imshow(win, vis)
        key = cv2.waitKey(30) & 0xFF
        if key in (ord('q'), ord('Q')):
            cv2.destroyAllWindows()
            sys.exit()

def convert_seqname_to_dateval(bagfullpath):
    seqdate = os.path.basename(os.path.dirname(bagfullpath))
    if 'aft' in seqdate or 'eve' in seqdate:
        val = float(seqdate[:-4]) + 0.5
    else:
        val = float(seqdate)
    return val

if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='LiDAR to Camera Projection')
    parser.add_argument('--bag_path', required=True)
    parser.add_argument('--lidar_topic', required=True)
    parser.add_argument('--image_topic', required=True)
    parser.add_argument('--gt_pose_file', required=True)
    parser.add_argument('--skip_seconds', type=float, default=0.0, help='Seconds to skip from bag start time')
    args = parser.parse_args()

    # Select calibration based on date
    split = float(20231207)
    d = convert_seqname_to_dateval(args.bag_path) # data，eg：20240113

    # extr：xt32_T_zed2leftcam
    if d > split:  # SUV platform
        extr = [0.0695283917427731, -0.008381612991474873, -0.17223038663727022,
                0.019635536507920586, 0.7097335839994078, -0.7039714840647372, -0.017799861603211602]
        K = np.array([[266.52, 0, 345.01], [0, 266.80, 189.99], [0, 0, 1]])
    else:  # handheld or ebike platform
        extr =  [0.07493894778041606, -0.16471796028449764, -0.09812580091216104,
                 -0.007712187968660904, -0.698602566500298,0.7154675436354508, 0.0010817764035590007]
        K = np.array([[533.04, 0, 659.02], [0, 533.60, 364.98], [0, 0, 1]])
    dist = np.array([-0.0567891, 0.032141, 0.000169392, -0.000430803, -0.0128658])

    proj = BagProjector(args.bag_path, args.gt_pose_file)
    T_lc = proj.pq_to_mat(extr)
    T_cl = np.linalg.inv(T_lc)

    bag_start = proj.bag.get_start_time()
    threshold = bag_start + args.skip_seconds

    lidar_buffer = deque(maxlen=1)

    for topic, msg, tm in proj.bag.read_messages(topics=[args.lidar_topic, args.image_topic]):
        ts = tm.to_sec()
        if ts < threshold:
            continue

        if topic == args.lidar_topic:
            # Store the LiDAR message and timestamp
            lidar_buffer.clear()
            lidar_buffer.append((msg, ts))

        elif topic == args.image_topic:
            if not lidar_buffer:
                continue

            lid_msg, lid_ts = lidar_buffer[0]
            img_msg, img_ts = msg, ts

            # interpolate the pose
            T1 = proj.interp_mat(img_ts)
            T2 = proj.interp_mat(lid_ts)
            if T1 is None or T2 is None:
                continue

            T_C1L2 = T_cl @ np.linalg.inv(T1) @ T2
            img = proj.img_msg_to_numpy(img_msg, args.image_topic)
            pts2d, pts3d = proj.project(lid_msg, img, T_C1L2, K, dist)
            proj.visualize(img, pts2d, pts3d)