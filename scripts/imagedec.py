#!/usr/bin/env python

import rospy
import cv2
import sys
sys.path.append('/home/ylc/cv_bridge_ws/install/lib/python3/dist-packages')

from cv_bridge import CvBridge
from sensor_msgs.msg import Image
import os

def capture_and_save_image(bridge, image_topic, save_path):
    image = rospy.wait_for_message(image_topic, Image)
    cv_image = bridge.imgmsg_to_cv2(image, desired_encoding="bgr8")
    
    # Save the image as a JPEG file
    cv2.imwrite(save_path, cv_image)
    print("Image saved as", save_path)

def main():
    rospy.init_node('imagedec', anonymous=True)
    bridge = CvBridge()
    image_topic = '/camera/color/image_raw'
    save_directory = '/home/ylc/Documents/C++/yolov5customDetector-main/yolov5/data_yt_1'
    if not os.path.exists(save_directory):
        os.makedirs(save_directory)

    rate = rospy.Rate(1)  # 1 Hz (capture an image every second)

    while not rospy.is_shutdown():
        timestamp = rospy.get_rostime().to_sec()
        image_filename = f"image_{timestamp}.jpg"
        save_path = os.path.join(save_directory, image_filename)

        capture_and_save_image(bridge, image_topic, save_path)

        rate.sleep()
    # image = rospy.wait_for_message('/camera/color/image_raw', Image)
    # cv_image = bridge.imgmsg_to_cv2(image, desired_encoding="bgr8")
    # print(cv_image)
    # cv2.imwrite('/home/ylc/Documents/C++/yolov5customDetector-main/yolov5/data_yt/image.jpg', cv_image)


if __name__ == '__main__':
    main()