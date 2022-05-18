import rclpy
from rclpy.node import Node

from std_msgs.msg import Empty
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

    self.toggle_manual_sub_ = self.create_subscription(
      Empty
      , '/lawn_mower/toggle_manual'
      , self.toggle_manual
      , 1)

    self.manual_control = True

  def process_joy(self, msg):
    if not self.manual_control:
      return
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

  def toggle_manual(self, msg):
    self.get_logger().info("Toggling from {} to {}".format(
      "MANUAL" if self.manual_control else "AUTO"
      , "AUTO" if self.manual_control else "MANUAL"
      ))
    self.manual_control = not self.manual_control

def main(args=None):
    rclpy.init(args=args)

    manual_control = ManualControl()
    
    rclpy.spin(manual_control)

    manual_control.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()