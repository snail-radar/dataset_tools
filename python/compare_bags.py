# -*-coding:utf-8-*-
import rosbag
import sys
import numpy as np

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
            message_count1 = bag1.get_message_count(topic)
            message_count2 = bag2.get_message_count(topic)

            if message_count1 != message_count2:
                report.write(f"Message count mismatch in topic {topic}.\n")
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
                if isinstance(v1, float):
                    if not np.isclose(v1, v2, atol=1e-6):
                        return False
                elif v1 != v2:
                    return False
        elif hasattr(value1, '__slots__'):  # Check if it's a ROS message
            if not compare_fields(value1, value2):
                return False
        else:
            if value1 != value2:
                return False
    return True

if __name__ == "__main__":
    if len(sys.argv) != 3:
        print("Usage: python compare_bags.py <path_to_bag1> <path_to_bag2>")
        sys.exit(1)
    
    bag1_path = sys.argv[1]
    bag2_path = sys.argv[2]
    
    compare_bags(bag1_path, bag2_path)
