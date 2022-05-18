import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, CancelResponse, GoalResponse

from std_msgs.msg import Empty
from lawn_mower_interfaces.action import Waypoint
from geometry_msgs.msg import Twist, PoseStamped, Point

import numpy as np
from time import sleep

from rclpy.executors import MultiThreadedExecutor

pose = PoseStamped().pose

def yaw_from_quaternion(q):
  return np.arctan2(2.0 * (q.w * q.z + q.x * q.y), q.w * q.w + q.x * q.x - q.y * q.y - q.z * q.z)

def get_yaw_error(target, current):
  error = target - current
  if abs(error) > np.pi:
    if error > 0:
      error -= 2 * np.pi
    else:
      error += 2 * np.pi
  return error

class PID():
  def __init__(self, p, i, d):
    self.p = p
    self.i = i
    self.d = d

    self.last_error = None
    self.sum_error = 0

  def get_control(self, error):
    if self.last_error is None:
      self.last_error = error
    diff_error = error - self.last_error
    self.sum_error += error
    self.last_error = error

    return self.p * error + self.i * self.sum_error + self.d * diff_error

  def reset(self):
    self.last_error = None
    self.sum_error = 0

class Navigator(Node):
  def __init__(self):
    super().__init__('navigator')

    self.waypoint_srv_ = ActionServer(
      self
      , Waypoint
      , 'waypoint'
      , execute_callback = self.process_waypoints
      , goal_callback = self.goal_callback
      , cancel_callback = self.cancel_callback
    )

    self.cmd_vel_pub_ = self.create_publisher(
      Twist
      , '/lawn_mower/cmd_vel'
      , 10
    )

    self.toggle_manual_pub_ = self.create_publisher(
      Empty
      , '/lawn_mower/toggle_manual'
      , 1)

    self.declare_parameters(
      namespace=''
      , parameters=[
        ('forward_p_gain', None),
        ('forward_i_gain', None),
        ('forward_d_gain', None),
        ('turn_p_gain', None),
        ('turn_i_gain', None),
        ('turn_d_gain', None),
        ('max_vel', 0.2),
        ('max_angular_vel', 0.5)
      ])

    self.forward_pid = PID(
      self.get_parameter('forward_p_gain').value
      , self.get_parameter('forward_i_gain').value
      , self.get_parameter('forward_d_gain').value
    )

    self.turn_pid = PID(
      self.get_parameter('turn_p_gain').value
      , self.get_parameter('turn_i_gain').value
      , self.get_parameter('turn_d_gain').value
    )
    self.go_straight = False

  def destroy(self):
    self.waypoint_srv_.destroy()
    super().destroy_node()

  def goal_callback(self, goal_request):
    self.get_logger().info("Received {} waypoints: {}".format(len(goal_request.waypoints), goal_request.waypoints))
    self.forward_pid.reset()
    self.turn_pid.reset()
    return GoalResponse.ACCEPT

  def cancel_callback(self, goal_handle):
    self.get_logger().info("Received cancel request")
    return CancelResponse.ACCEPT

  async def process_waypoints(self, goal_handle):
    toggle_manual_msg = Empty()
    self.toggle_manual_pub_.publish(toggle_manual_msg)
    #drive to waypoint
    goal = goal_handle.request
    while len(goal.waypoints):
      self.forward_pid.reset()
      self.turn_pid.reset()
      waypoint = goal.waypoints.pop(0)
      self.get_logger().info("Driving to waypoint at x={:.2f} y={:.2f}".format(waypoint.x, waypoint.y))
      feedback_msg = Waypoint.Feedback()

      dist = ((pose.position.x - waypoint.x) ** 2 + (pose.position.y - waypoint.y) **2) ** 0.5
      feedback_msg.distance = dist
      while dist > 0.05:
        msg = Twist()
        target_yaw = np.arctan2(waypoint.y - pose.position.y, waypoint.x - pose.position.x)
        curr_yaw = yaw_from_quaternion(pose.orientation)
        yaw_error = get_yaw_error(target_yaw, curr_yaw)

        self.get_logger().info("yaw_error={:.3f} target_yaw={:.3f} curr_yaw={:.3f}".format(yaw_error, target_yaw, curr_yaw))
        msg.angular.z = min(self.turn_pid.get_control(yaw_error), self.get_parameter('max_angular_vel').value)
        if abs(yaw_error) > 0.05:
          self.go_straight = False
        if abs(yaw_error) < 0.01:
          self.go_straight = True
        if self.go_straight:
          msg.linear.x = min(self.forward_pid.get_control(dist), self.get_parameter('max_vel').value)
        self.cmd_vel_pub_.publish(msg)
        #self.get_logger().info("cmd_msg={}".format(msg))

        dist = ((pose.position.x - waypoint.x) ** 2 + (pose.position.y - waypoint.y) **2) ** 0.5
        feedback_msg.distance = dist
        goal_handle.publish_feedback(feedback_msg)
        sleep(0.1)
      
      msg = Twist()
      msg.linear.x = 0.0
      msg.angular.z = 0.0
      self.cmd_vel_pub_.publish(msg)

      feedback_msg.distance = dist
      goal_handle.publish_feedback(feedback_msg)

    goal_handle.succeed()
    self.get_logger().info("Successfully travelled to all waypoints!")

    result = Waypoint.Result()
    result.success = True
    self.toggle_manual_pub_.publish(toggle_manual_msg)

    return result

class PoseSubscriber(Node):
  def __init__(self):
    super().__init__('pose_subscriber')
    self.pose_sub_ = self.create_subscription(
      PoseStamped
      , "/lawn_mower/pose"
      , self.process_pose
      , 10)

  def process_pose(self, msg):
    global pose
    pose = msg.pose

def main(args=None):
  rclpy.init(args=args)

  try:
    navigator = Navigator()
    pose_sub = PoseSubscriber()

    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(navigator)
    executor.add_node(pose_sub)

    try:
      executor.spin()

    finally:
      # Destroy the node explicitly
      # (optional - otherwise it will be done automatically
      # when the garbage collector destroys the node object)
      executor.shutdown()
      navigator.destroy_node()
      pose_sub.detroy_node()
  finally:
    rclpy.shutdown()

if __name__ == '__main__':
  main()
