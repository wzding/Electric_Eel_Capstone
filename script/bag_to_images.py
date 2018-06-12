#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
Extract images from a rosbag.
python bag_to_images.py --bag_file just_traffic_light.bag --output_dir images/  --file_prefix just_traffic_light_
python bag_to_images.py --bag_file loop_with_traffic_light.bag --output_dir images/  --file_prefix loop_with_traffic_light_
"""
import os
import argparse
import cv2
import rosbag
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

parser = argparse.ArgumentParser(description="Extract images from a ROS bag.")
parser.add_argument("--bag_file", help="Input ROS bag.")
parser.add_argument("--output_dir", help="Output directory.")
parser.add_argument("--file_prefix", help="file prefix.")
args = parser.parse_args()
bag_file = args.bag_file
output_dir = args.output_dir
file_prefix = args.file_prefix

def get_image_topic(bag):
    info = bag.get_type_and_topic_info()
    # info is a tuple
    image_topic = info[1].keys()[0]
    return image_topic

def main():
    """Extract a folder of images from a rosbag.
    """
    print("Extract images from %s into %s" % (bag_file, output_dir))
    bridge = CvBridge()
    count = 0
    bag = rosbag.Bag(bag_file, "r")
    image_topic = get_image_topic(bag)
    for topic, msg, t in bag.read_messages(topics=[image_topic]):
        cv_img = bridge.imgmsg_to_cv2(msg, desired_encoding="rgb8")
        # color image ??
        # cv2.imwrite(os.path.join(output_dir, "frame%06i.png" % count),
        #     cv2.cvtColor(cv_img, cv2.COLOR_GRAY2RGB))
        cv2.imwrite(os.path.join(output_dir, file_prefix + "%04i.jpeg" % count), cv_img)
        print("Wrote image %i" % count)
        count += 1
    bag.close()
    return

if __name__ == '__main__':
    main()
