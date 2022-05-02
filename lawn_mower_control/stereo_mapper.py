import rclpy
from rclpy.node import Node
import message_filters

from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image

from cv_bridge import CvBridge
import cv2
import matplotlib.pyplot as plt

import numpy as np

import pyransac3d as pyrsc


class LawnMowerMapper(Node):
    def __init__(self):
        super().__init__("lawn_mower_mapper")
        stereo_images = [
          message_filters.Subscriber(self, Image, 'stereo_camera/left/image_raw')
          , message_filters.Subscriber(self, Image, 'stereo_camera/right/image_raw')
        ]

        self.camera_sub_ = message_filters.TimeSynchronizer(
            stereo_images
            , 10
        )
        self.camera_sub_.registerCallback(self.process_stereo)
        self.br = CvBridge()
        self.stereo = cv2.StereoBM_create()
        self.stereo.setNumDisparities(128)
        self.stereo.setBlockSize(11)
        self.focal_length = 238.35
        self.baseline_width = 0.1

    def process_stereo(self, left_image, right_image):
        left_image = self.br.imgmsg_to_cv2(left_image)
        right_image = self.br.imgmsg_to_cv2(right_image)
        left_gray = cv2.cvtColor(left_image, cv2.COLOR_BGR2GRAY)
        right_gray = cv2.cvtColor(right_image, cv2.COLOR_BGR2GRAY)
        disparity = self.stereo.compute(left_gray, right_gray)
        cy, cx = [s /2 for s in left_image.shape[:2]]
        points = []
        coordinates = []
        for y, x in zip(*np.where(disparity > 0)):
          d = disparity[y, x] / 16.
          points.append([self.baseline_width * (x - cx) / d
                        ,self.baseline_width * (y - cy) / d
                        ,self.baseline_width * self.focal_length / d])
          coordinates.append((x,y))
        points = np.array(points)
        plane1 = pyrsc.Plane()
        best_eq, inliers = plane1.fit(points - np.mean(points, axis=0),0.01)
        new_disp = np.zeros(disparity.shape)
        for i in inliers:
          x, y = coordinates[i]
          new_disp[y, x] = disparity[y, x]
        breakpoint()


def main(args=None):
    rclpy.init(args=args)

    lawn_mower_mapper = LawnMowerMapper()
    
    rclpy.spin(lawn_mower_mapper)

    lawn_mower_mapper.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
        
        