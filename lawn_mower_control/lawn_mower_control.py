import rclpy
from rclpy.node import Node
import message_filters

from geometry_msgs.msg import Twist

import numpy as np


class LawnMowerControl(Node):
    def __init__(self):
        super().__init__("lawn_mower_control")
        self.lawn_mower_tfs = []
        for i in range(3):
            self.lawn_mower_tfs.append(
                message_filters.Subscriber(
                    self
                    , TFMessage
                    , 'lawn_mower{}/tf'.format(i)
                    )
                )
        self.tf_sub_ = message_filters.TimeSynchronizer(
            self.lawn_mower_tfs
            , 10
        )
        self.tf_sub_.registerCallback(self.process_tfs)

    def process_tfs(self, *lawn_mower_tfs):
        points = [(tf.translation.x, tf.translation.y) for tf in lawn_mower_tfs]
        center, radius = get_circle(points)
        
  


