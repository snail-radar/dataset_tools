import os
import cv2
import open3d as o3d
import struct


hesai_fieldtypecodes = ('f', 'f', 'f', 'f', 'd', 'H')
hesai_unpack_numbytes = (4, 4, 4, 4, 8, 2)

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
            invaliddata = False
            for i, field in enumerate(fields):
                if "xt32" in filename:
                    value_bytes = file.read(hesai_unpack_numbytes[i])
                else:
                    value_bytes = file.read(4)
                if not value_bytes:
                    invaliddata = True
                    break
                if "xt32" in filename:
                    value = struct.unpack(hesai_fieldtypecodes[i], value_bytes)[0]
                else:
                    value = struct.unpack('f', value_bytes)[0]
                point_data[field] = value
            if not point_data:
                break
            if invaliddata:
                print("Warn: invalid data {} at {} of {}".format(point_data, len(points), filename))
            points.append([point_data[field] for field in fields])

        return fields, points

def initialize_visualizers(topics):
    """
    Initialize Open3D visualizers for multiple topics.
    """
    visualizers = {}
    point_clouds = {}
    window_positions = [
        (0, 0), (470, 0), (870, 0), (1270, 0)  # Adjust positions for point cloud windows
    ]

    for i, topic in enumerate(topics):
        vis = o3d.visualization.Visualizer()
        vis.create_window(window_name=f"{topic} Viewer", width=400, height=300, left=window_positions[i][0], top=window_positions[i][1])
        vis.get_render_option().point_size = 2.0  # Set point size for better visualization
        vis.get_view_control().set_zoom(0.8)
        if i < len(window_positions):
            x, y = window_positions[i]
            os.system(f"xdotool search --name '{topic} Viewer' windowmove {x} {y}")
        pc = o3d.geometry.PointCloud()
        vis.add_geometry(pc)
        visualizers[topic] = vis
        point_clouds[topic] = pc

    return visualizers, point_clouds

def update_visualizer(vis, pc, points):
    """
    Update the Open3D visualizer with new point cloud data.
    """
    points_xyz = [[p[0], p[1], p[2]] for p in points]  # Extract x, y, z coordinates
    # print(f"Number of points: {len(points_xyz)}")
    pc.points = o3d.utility.Vector3dVector(points_xyz)
    pc.colors = o3d.utility.Vector3dVector([[0, 0, 1] for _ in points_xyz])  # Set color to blue

    vis.clear_geometries()
    vis.add_geometry(pc)
    vis.poll_events()
    vis.update_renderer()

def load_image_timestamps_and_paths(folder_path, image_type):
    """
    Load image timestamps and paths from a folder.
    """
    image_folder = os.path.join(folder_path, image_type)
    image_bns = [fn for fn in os.listdir(image_folder) if fn.lower().endswith('.jpg')]
    image_paths = [os.path.join(image_folder, bn) for bn in image_bns]
    timestamps = [os.path.splitext(bn)[0] for bn in image_bns]
    return timestamps, image_paths

def display_images(left_image_path, right_image_path):
    """
    Display left and right images in separate OpenCV windows.
    """
    if left_image_path and os.path.exists(left_image_path):
        left_image = cv2.imread(left_image_path)
        cv2.imshow("Left Image", left_image)
        cv2.moveWindow("Left Image", 0, 380)  # Position the left image window

    if right_image_path and os.path.exists(right_image_path):
        right_image = cv2.imread(right_image_path)
        cv2.imshow("Right Image", right_image)
        cv2.moveWindow("Right Image", 720, 380)  # Position the right image window

    cv2.waitKey(1)  # Display for 50ms

def synchronize_and_display(folder_path, query_topics, left_images, right_images):
    """
    Synchronize and display point clouds and images.
    """
    # Load point cloud file paths and timestamps
    topic_files = {}
    topic_timestamps = {}
    topics = []
    for topic in query_topics:
        pcd_folder = os.path.join(folder_path, topic)
        if not os.path.exists(pcd_folder):
            print(f"Point cloud folder not found: {pcd_folder}")
            continue

        files = [f for f in os.listdir(pcd_folder) if f.endswith('.pcd')]
        files.sort(key=lambda x: float(x.split('.')[0]) + float(x.split('.')[1]) / 1e9)  # Sort by timestamp
        topic_files[topic] = [os.path.join(pcd_folder, f) for f in files]
        topic_timestamps[topic] = [float(f.split('.')[0]) + float(f.split('.')[1]) / 1e9 for f in files]
        topics.append(topic)

    # Initialize visualizers for point clouds
    visualizers, point_clouds = initialize_visualizers(topics)

    # Load image paths and timestamps
    left_timestamps, left_paths = left_images
    right_timestamps, right_paths = right_images

    # Convert image timestamps to float for synchronization
    left_timestamps = [float(ts) for ts in left_timestamps]
    right_timestamps = [float(ts) for ts in right_timestamps]

    # Synchronize timestamps
    all_timestamps = sorted(set(left_timestamps) | set(right_timestamps) | 
        set().union(*(set(topic_timestamps[topic]) for topic in topics)))

    print("Press 'Q' to quit.")
    for timestamp in all_timestamps:
        print(f"Displaying data for timestamp: {timestamp}")

        # Display point clouds for each topic
        for topic in topics:
            if timestamp in topic_timestamps[topic]:
                idx = topic_timestamps[topic].index(timestamp)
                file_path = topic_files[topic][idx]
                _, points = read_pcd_file(file_path)
                if topic == 'xt32':
                    # Downsample the point cloud for better performance
                    points = points[::10]
                update_visualizer(visualizers[topic], point_clouds[topic], points)

        # Display images
        left_image_path = right_image_path = None
        if timestamp in left_timestamps:
            left_idx = left_timestamps.index(timestamp)
            left_image_path = left_paths[left_idx]
        if timestamp in right_timestamps:
            right_idx = right_timestamps.index(timestamp)
            right_image_path = right_paths[right_idx]

        display_images(left_image_path, right_image_path)

        key = cv2.waitKey(1)  # Delay for 2ms
        if key == ord('q'):
            break

    # Close Open3D visualizers
    for vis in visualizers.values():
        vis.destroy_window()

    # Close OpenCV windows
    cv2.destroyAllWindows()

def display_data(target_folder):
    """
    Display data (images and point clouds) from a specific folder
    """
    if not os.path.exists(target_folder):
        print(f"Folder not found: {target_folder}")
        return

    print(f"Displaying data from folder: {target_folder}")

    # Define topics for point clouds
    topics = ['ars548/points', 'eagleg7/enhanced', 'eagleg7/pcl', 'xt32']

    # Load left and right images
    left_images = load_image_timestamps_and_paths(target_folder, 'zed2i/left')
    right_images = load_image_timestamps_and_paths(target_folder, 'zed2i/right')

    # Synchronize and display all data
    synchronize_and_display(target_folder, topics, left_images, right_images)

if __name__ == "__main__":
    import argparse

    parser = argparse.ArgumentParser(description="View synchronized point clouds and images.")
    parser.add_argument("folder_path", help="Path to the data folder. E.g., /data/snail/20231019/data2")
    args = parser.parse_args()

    folder_path = args.folder_path

    if not os.path.exists(folder_path):
        print(f"Folder not found: {folder_path}")
        exit(1)

    display_data(folder_path)
