#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Image, CameraInfo
import sys
sys.path.append('/home/ylc/cv_bridge_ws/install/lib/python3/dist-packages')
from cv_bridge import CvBridge
import cv2
import open3d as o3d
import numpy as np


class ROSOpen3DPointCloudGenerator:
    def __init__(self):
        rospy.init_node('ros_open3d_pointcloud_generator', anonymous=True)
        self.bridge = CvBridge()

        # Subscribe to RGB and depth image topics
        self.rgb = rospy.wait_for_message('/camera/color/image_raw', Image)
        self.depth = rospy.wait_for_message('/camera/aligned_depth_to_color/image_raw', Image)

        # Camera intrinsics
        self.fx = 606.31005859375
        self.fy = 606.111572265625
        self.cx = 324.8408508300781
        self.cy = 248.92527770996094

        self.rgb_image = self.bridge.imgmsg_to_cv2(self.rgb, desired_encoding="rgb8")
        self.depth_image = self.bridge.imgmsg_to_cv2(self.depth)
        self.depth_image = self.depth_image.astype(np.uint16)

        # Initialize Open3D visualizer
        self.vis = o3d.visualization.Visualizer()
        self.vis.create_window()

        # Convert depth image to point cloud
        intrinsics = o3d.camera.PinholeCameraIntrinsic(
            width=640,
            height=480,
            fx=self.fx,
            fy=self.fy,
            cx=self.cx,
            cy=self.cy
        )

        depth_image = o3d.geometry.Image(np.array(self.depth_image))
        rgbd_image = o3d.geometry.RGBDImage.create_from_color_and_depth(
            o3d.geometry.Image(np.array(self.rgb_image)),
            depth_image, convert_rgb_to_intensity=False
        )

        pcd = o3d.geometry.PointCloud.create_from_rgbd_image(rgbd_image, intrinsics)

        self.vis.clear_geometries()
        self.vis.add_geometry(pcd)
        # self.vis.update_geometry(pcd)
        # self.vis.poll_events()
        # self.vis.update_renderer()
        o3d.visualization.draw_geometries([pcd])


if __name__ == '__main__':
    try:
        pointcloud_generator = ROSOpen3DPointCloudGenerator()
        # rospy.spin()
    except rospy.ROSInterruptException:
        pass