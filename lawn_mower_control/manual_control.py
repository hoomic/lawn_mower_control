import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy

class ManualControl(Node):
  def __init__(self):
    super().__init__("manual_control")
    self.joystick_sub_ = self.create_subscription(
      Joy
      , '/joy'
      , self.process_joy
      , 10)

    self.cmd_vel_pub = self.create_publisher(
      Twist
      , '/lawn_mower/cmd_vel'
      , 10)

  def process_joy(self, msg):
    yaw, _, lt, _, _, rt, _, _ = msg.axes
    forward = (-rt + lt) * 0.1
    if abs(forward) < 1e-3:
      forward = 0.0
    if abs(yaw) < 1e-3:
      yaw = 0.0
    vel_msg = Twist()
    vel_msg.linear.x = forward
    vel_msg.angular.z = yaw * 0.5
    self.cmd_vel_pub.publish(vel_msg)

def main(args=None):
    rclpy.init(args=args)

    manual_control = ManualControl()
    
    rclpy.spin(manual_control)

    manual_control.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()