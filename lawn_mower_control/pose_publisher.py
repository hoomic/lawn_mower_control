import rclpy
from rclpy.node import Node

from geometry_msgs.msg import PoseStamped
from tf2_msgs.msg import TFMessage

from quaternion import quaternion

def quaternions_from_transform(transform):
  t = transform.transform.translation
  r = transform.transform.rotation
  trans = quaternion(0, t.x, t.y, t.z)
  rot = quaternion(r.w, r.x, r.y, r.z)
  return trans, rot

class PosePublisher(Node):
  """
  This node takes the odometry topic and the tf from the map coordinate system to the 
  odometry coordinate system and combines them to publish the pose of the robot in the
  map coordinate system.
  """
  def __init__(self):
    super().__init__("pose_publisher")
    self.tf_sub_ = self.create_subscription(
      TFMessage
      , "/tf"
      , self.process_tf
      , 10
      )

    self.pose_pub_ = self.create_publisher(
      PoseStamped
      , "/lawn_mower/pose"
      , 10)

    self.map_to_odom = None
    self.odom_to_base = None

    self.timer = self.create_timer(1./60, self.publish_pose)

  def process_tf(self, msg):
    for transform in msg.transforms:
      if transform.header.frame_id == 'map' and transform.child_frame_id == 'odom':
        self.map_to_odom = quaternions_from_transform(transform)
      if transform.header.frame_id == 'odom' and transform.child_frame_id == 'base_link':
        self.odom_to_base = quaternions_from_transform(transform)

  def publish_pose(self):
    if self.map_to_odom is None or self.odom_to_base is None:
      return
    odom_trans, odom_rot = self.map_to_odom
    base_trans, base_rot = self.odom_to_base

    position = odom_rot * base_trans * odom_rot.conjugate() + odom_trans
    orientation = odom_rot * base_rot

    pose = PoseStamped()
    pose.header.stamp = self.get_clock().now().to_msg()
    pose.header.frame_id = 'map'
    pose.pose.position.x = position.x
    pose.pose.position.y = position.y
    pose.pose.position.z = position.z
    pose.pose.orientation.x = orientation.x
    pose.pose.orientation.y = orientation.y
    pose.pose.orientation.z = orientation.z
    pose.pose.orientation.w = orientation.w

    self.pose_pub_.publish(pose)

def main(args=None):
  rclpy.init(args=args)

  pose_publisher = PosePublisher()

  rclpy.spin(pose_publisher)

  # Destroy the node explicitly
  # (optional - otherwise it will be done automatically
  # when the garbage collector destroys the node object)
  pose_publisher.destroy_node()
  rclpy.shutdown()

if __name__ == '__main__':
  main()
