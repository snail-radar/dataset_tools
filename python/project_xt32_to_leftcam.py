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

def load_pose_file(gt_pose_file):
    try:
        return np.loadtxt(gt_pose_file, delimiter=',')
    except ValueError:
        # Fall back to whitespace-separated
        return np.loadtxt(gt_pose_file)  # default delimiter=None (whitespace)

class BagProjector:
    def __init__(self, bag_path, gt_pose_file):
        # Load ROS bag and distortion coefficients
        self.bag = rosbag.Bag(bag_path)
        if os.path.isfile(gt_pose_file):
            # time,x,y,z,qx,qy,qz,qw
            data = load_pose_file(gt_pose_file)
            self.times = data[:, 0]
            self.poses = data[:, 1:8]
        else:
            self.times = None
            self.poses = None

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
        if self.times is None or t < self.times[0] or t > self.times[-1]:
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

    def visualize(self, img, pts2d, pts_cam, wait_ms, circle_radius=3):
        if pts_cam.ndim != 2 or pts_cam.shape[1] < 3:
            raise ValueError("`pts_cam` must be a 2D array with at least 3 columns (x, y, z).")
        vis = img.copy()
        depths = pts_cam[:, 2]
        nd = (depths - depths.min()) / (depths.max() - depths.min())
        colors = (plt.cm.viridis(nd)[:, :3] * 255).astype(int)
        # colors = (plt.cm.turbo(nd)[:, :3] * 255).astype(int)
        for (x, y), c in zip(pts2d, colors):
            cv2.circle(vis, (int(x), int(y)), circle_radius, (int(c[2]), int(c[1]), int(c[0])), -1)
        win = 'Projection'
        cv2.imshow(win, vis)
        key = cv2.waitKey(wait_ms) & 0xFF
        if key in (ord('q'), ord('Q')):
            cv2.destroyAllWindows()
            sys.exit()
        return vis

def convert_seqname_to_dateval(bagfullpath):
    seqdate = os.path.basename(os.path.dirname(bagfullpath))
    if 'aft' in seqdate or 'eve' in seqdate:
        val = float(seqdate[:-4]) + 0.5
    else:
        val = float(seqdate)
    return val

if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='Point cloud2 to Camera Projection')
    parser.add_argument('bag_path', help='Path to the ROS bag file')
    parser.add_argument('--pc_topic', help="point cloud2 topic", default="/hesai/pandar")
    parser.add_argument('--image_topic', default="/zed2i/zed_node/left_raw/image_raw_gray/compressed")
    parser.add_argument('--gt_pose_file', default="path to the full traj file. optional.")
    parser.add_argument('--skip_seconds', type=float, default=0.0, help='Seconds to skip from bag start time')
    parser.add_argument('--wait_ms', type=int, default=1, help='Wait time in ms for visualization')
    args = parser.parse_args()

    # Select calibration based on date
    split = float(20231207)
    d = convert_seqname_to_dateval(args.bag_path) # data，eg：20240113

    # extr：pcframe_T_zed2leftcam
    extr = None
    if d > split:  # SUV platform
        if 'hesai' in args.pc_topic:
            extr = [0.0695283917427731, -0.008381612991474873, -0.17223038663727022,
                    0.019635536507920586, 0.7097335839994078, -0.7039714840647372, -0.017799861603211602]
        elif 'radar' in args.pc_topic:
            extr = [0.0528194205985374, 0.0732363401241965, -0.078559196101828, 
                    -0.488837681067262, 0.520090547638786, -0.496101256310584, 0.494395678897611]
        K = np.array([[266.52, 0, 345.01], [0, 266.80, 189.99], [0, 0, 1]])
    else:  # handheld or ebike platform
        if 'hesai' in args.pc_topic:
            extr =  [0.07493894778041606, -0.16471796028449764, -0.09812580091216104,
                    -0.007712187968660904, -0.698602566500298,0.7154675436354508, 0.0010817764035590007]
        elif 'radar' in args.pc_topic:
            extr = [ 0.0528194205985374, 0.0732363401241965, -0.078559196101828, -0.488945904875287, 
                    0.503830346308189, -0.492430762087489, 0.514391707548432]
        K = np.array([[533.04, 0, 659.02], [0, 533.60, 364.98], [0, 0, 1]])
    print(f'topic {args.pc_topic}, extr {extr}')
    dist = np.array([-0.0567891, 0.032141, 0.000169392, -0.000430803, -0.0128658])

    proj = BagProjector(args.bag_path, args.gt_pose_file)
    T_lc = proj.pq_to_mat(extr)
    T_cl = np.linalg.inv(T_lc)

    bag_start = proj.bag.get_start_time()
    threshold = bag_start + args.skip_seconds

    pc_buffer = deque(maxlen=1)

    fourcc = cv2.VideoWriter_fourcc(*'mp4v')   # codec for .mp4
    fps = 25                                   # adjust to your frame rate
    height, width = -1, -1
    out = None
    save_video = False

    for topic, msg, tm in proj.bag.read_messages(topics=[args.pc_topic, args.image_topic]):
        ts = tm.to_sec()
        if ts < threshold:
            continue

        if topic == args.pc_topic:
            # Store the point cloud message and timestamp
            pc_buffer.clear()
            pc_buffer.append((msg, ts))

        elif topic == args.image_topic:
            if not pc_buffer:
                continue

            lid_msg, lid_ts = pc_buffer[0]
            img_msg, img_ts = msg, ts

            T1 = proj.interp_mat(img_ts)
            T2 = proj.interp_mat(lid_ts)
            if T1 is None or T2 is None:
                T_C1L2 = T_cl
            else:
                T_C1L2 = T_cl @ np.linalg.inv(T1) @ T2

            img = proj.img_msg_to_numpy(img_msg, args.image_topic)
            pts2d, pts3d = proj.project(lid_msg, img, T_C1L2, K, dist)
            vis = proj.visualize(img, pts2d, pts3d, wait_ms=args.wait_ms)
            if out is None and save_video:
                height, width = vis.shape[:2]
                out = cv2.VideoWriter('projection_output.mp4', fourcc, fps, (width, height))
            if out is not None:
                out.write(vis)
    if out is not None:
        out.release()
    cv2.destroyAllWindows()
    sys.exit()
