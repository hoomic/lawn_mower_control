import rclpy
from rclpy.node import Node

from std_msgs.msg import Empty
from nav_msgs.msg import Odometry, Path
from sensor_msgs.msg import Joy
from geometry_msgs.msg import PoseStamped
from tf2_msgs.msg import TFMessage

from .path_planner import Zone

from quaternion import quaternion

class ZoneMapper(Node):
  def __init__(self):
    super().__init__("zone_mapper")
    self.tf_sub_ = self.create_subscription(
      TFMessage
      , "/tf"
      , self.process_tf
      , 10
      )

    self.odom_sub_ = self.create_subscription(
      Odometry
      , "/visual_slam/tracking/odometry"
      , self.process_odom
      , 10
    )


    self.joystick_sub_ = self.create_subscription(
      Joy
      , '/joy'
      , self.process_joy
      , 10
      )

    self.zone = None
    self.map_to_odom_rot = None
    self.map_to_odom_trans = None

    self.path_pub_ = self.create_publisher(
      Path
      , "/lawn_mower/zones"
      , 10
      )

    self.perimeter_mapped_pub_ = self.create_publisher(Empty, "/lawn_mower/perimeter_mapped", 1)

    self.a_off_seen = True

  def process_joy(self, msg):
    a_press, b_press, _, _, _, _, _, _, home_press, *_ = msg.buttons
    if a_press and self.a_off_seen and self.zone is None:
      self.a_off_seen = False
      self.zone = Zone()
      self.get_logger().info("Tracking new zone")
    else:
      self.a_off_seen = True
    if b_press and self.zone is not None:
      self.get_logger().info("Finished tracking zone")
      msg = self.zone .get_path_message()
      self.path_pub_.publish(msg)
      self.zone = None
    if home_press:
      if self.home_press_start is None:
        self.home_press_start = msg.header.stamp.sec
        self.last_time_held = 0
        self.get_logger().info("Hold for 5s to exit")
      else:
        time_held = msg.header.stamp.sec - self.home_press_start
        if time_held >= 5:
          self.get_logger().info("Exiting")
          self.exit()
        elif time_held != self.last_time_held:
          self.get_logger().info("Hold for {}s to exit".format(5 - time_held))
          self.last_time_held = time_held
    else:
      self.home_press_start = None

  def process_tf(self, msg):
    if self.zone is None:
      return
    for transform in msg.transforms:
      if transform.header.frame_id == 'map' and transform.child_frame_id == 'odom':
        t = transform.transform.translation
        r = transform.transform.rotation
        self.map_to_odom_rot = quaternion(r.w, r.x, r.y, r.z)
        self.map_to_odom_trans = quaternion(0, t.x, t.y, t.z)
        break

  def process_odom(self, msg):
    if self.zone is None or self.map_to_odom_rot is None:
      return
    pose = PoseStamped()
    p = msg.pose.pose.position
    pos = quaternion(0, p.x, p.y, p.z)
    map_pos = self.map_to_odom_rot * pos * self.map_to_odom_rot.conjugate() + self.map_to_odom_trans
    pose.pose.position.x = map_pos.x
    pose.pose.position.y = map_pos.y
    pose.pose.position.z = map_pos.z
    pose.header = msg.header
    pose.header.frame_id = 'map'
    if not self.zone.add_perimeter(pose):
      self.get_logger().info("Zone closure detected")
      msg = self.zone .get_path_message()
      self.path_pub_.publish(msg)
      self.zone = None

  def exit(self):
    msg = Empty()
    self.perimeter_mapped_pub_.publish(msg)
    self.destroy_node()

def main(args=None):
  rclpy.init(args=args)

  zone_mapper = ZoneMapper()

  rclpy.spin(zone_mapper)

  # Destroy the node explicitly
  # (optional - otherwise it will be done automatically
  # when the garbage collector destroys the node object)
  zone_mapper.destroy_node()
  rclpy.shutdown()

if __name__ == '__main__':
  main()
