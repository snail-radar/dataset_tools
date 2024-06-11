# -*-coding:utf-8-*-
import rosbag
import sys
import numpy as np
import cv2
from sensor_msgs.msg import PointCloud2
import sensor_msgs.point_cloud2 as pc2

ignore_fields = ['position_covariance_type', "child_frame_id"] # "point_step", "row_step"
ignore_topics = ['/radar_trk']

def compare_bags(bag1_path, bag2_path):
    bag1 = rosbag.Bag(bag1_path)
    bag2 = rosbag.Bag(bag2_path)
    
    topics1 = bag1.get_type_and_topic_info()[1].keys()
    topics2 = bag2.get_type_and_topic_info()[1].keys()
    
    topics1_set = set(topics1)
    topics2_set = set(topics2)
    all_match = True
    with open('mismatch_report.txt', 'w') as report:
        if topics1_set != topics2_set:
            missing_in_bag1 = topics2_set - topics1_set
            missing_in_bag2 = topics1_set - topics2_set
            if missing_in_bag1:
                report.write(f"Topics missing in bag1: {', '.join(missing_in_bag1)}\n")
            if missing_in_bag2:
                report.write(f"Topics missing in bag2: {', '.join(missing_in_bag2)}\n")
            all_match = False
        
        for topic in topics1_set.intersection(topics2_set):
            if topic in ignore_topics:
                continue
            message_count1 = bag1.get_message_count(topic)
            message_count2 = bag2.get_message_count(topic)

            if message_count1 != message_count2:
                report.write(f"Message count mismatch in topic {topic}.\n")
                print(f"Message count mismatch in topic {topic}. {message_count1} != {message_count2}")
                all_match = False
                continue

            messages1 = bag1.read_messages(topic)
            messages2 = bag2.read_messages(topic)
            for i, (msg1, msg2) in enumerate(zip(messages1, messages2)):
                _, msg1, _ = msg1
                _, msg2, _ = msg2
                if hasattr(msg1, 'header') and hasattr(msg2, 'header'):
                    t1 = msg1.header.stamp.to_sec()
                    t2 = msg2.header.stamp.to_sec()
                    if not np.isclose(t1, t2, atol=1e-2):  # Considering precision issues
                        report.write(f"Timestamp mismatch in topic {topic} at index {i}. {t1} != {t2}\n")
                        all_match = False
                        continue

                if topic == "/hesai/pandar":
                    if not compare_hesai(msg1, msg2):
                        report.write(f"Fields mismatch in topic {topic} at index {i}.\n")
                        all_match = False
                    continue

                if not compare_fields(msg1, msg2):
                    report.write(f"Fields mismatch in topic {topic} at index {i}.\n")
                    all_match = False
        if all_match:
            report.write("All topics and messages match.\n")
    return all_match

def compare_fields(msg1, msg2):
    if type(msg1) != type(msg2):
        return False
    for field in msg1.__slots__:
        if field == 'header':
            # compare header.stamp, header.frame_id
            if msg1.header.frame_id != msg2.header.frame_id:
                return False
            if msg1.header.stamp.to_nsec() != msg2.header.stamp.to_nsec():
                return False
            continue
        if field == 'status':
            if msg1.status.status != msg2.status.status:
                return False
            continue
        if field.endswith('covariance') or field in ignore_fields:
            continue
        if not hasattr(msg2, field):
            return False
        value1 = getattr(msg1, field)
        value2 = getattr(msg2, field)
        if isinstance(value1, float):
            if not np.isclose(value1, value2, atol=1e-6):
                return False
        elif isinstance(value1, (np.ndarray, list, tuple)):
            if len(value1) != len(value2):
                return False
            for v1, v2 in zip(value1, value2):
                if isinstance(v1, float) :
                    if not np.isclose(v1, v2, atol=1e-6):
                        return False
                elif v1 != v2:
                    if field=="fields" and v1.name==v2.name and v1.datatype==v2.datatype and v1.count==v2.count:
                        continue
                    return False
        elif hasattr(value1, '__slots__'):  # Check if it's a ROS message
            if not compare_fields(value1, value2):
                return False
        else:
            if value1 != value2:
                if field == 'data' and "CompressedImage" in str(type(msg1)):
                    np_arr1 = np.frombuffer(value1, np.uint8)
                    np_arr2 = np.frombuffer(value2, np.uint8)
                    img1 = cv2.imdecode(np_arr1, cv2.IMREAD_COLOR)
                    img2 = cv2.imdecode(np_arr2, cv2.IMREAD_COLOR)
                    if img1 is not None and img2 is not None:
                        diff_img = cv2.absdiff(img1, img2)
                        mean_diff = np.mean(diff_img)
                        if mean_diff > 1:
                            print("Mean diff: ", mean_diff, "lager then 1")
                            return False
                        else:
                            return True
                return False
    return True

def compare_hesai(msg1, msg2):
    """Compare two Hesai PointCloud2 messages."""
    if msg1.header.frame_id != msg2.header.frame_id:
        return False
    if msg1.header.stamp.to_nsec() != msg2.header.stamp.to_nsec():
        return False

    if msg1.height != msg2.height:
        return False
    if msg1.width != msg2.width:
        return False
    if msg1.is_dense != msg2.is_dense:
        return False

    if len(msg1.fields) != len(msg2.fields):
        return False

    for f1, f2 in zip(msg1.fields, msg2.fields):
        if f1.name != f2.name or f1.offset != f2.offset or f1.datatype != f2.datatype or f1.count != f2.count:
            return False

    fields_list1 = {f.name: f for f in msg1.fields}
    fields_list2 = {f.name: f for f in msg2.fields}
    points_list1 = []
    points_list2 = []
    for point in pc2.read_points(msg1, field_names=fields_list1.keys(), skip_nans=True):
        points_list1.append(point)
    for point in pc2.read_points(msg2, field_names=fields_list2.keys(), skip_nans=True):
        points_list2.append(point)
    
    if len(points_list1) != len(points_list2):
        return False
    for p1, p2 in zip(points_list1, points_list2):
        # compare two turple
        if p1 != p2:
            return False

    return True

if __name__ == "__main__":
    if len(sys.argv) != 3:
        print("Usage: python compare_bags.py <path_to_bag1> <path_to_bag2>")
        sys.exit(1)
    
    bag1_path = sys.argv[1]
    bag2_path = sys.argv[2]
    
    compare_bags(bag1_path, bag2_path)
