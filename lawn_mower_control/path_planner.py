import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

from std_msgs.msg import Empty
from nav_msgs.msg import Path
from geometry_msgs.msg import Point, Vector3
from visualization_msgs.msg import Marker, MarkerArray
from lawn_mower_interfaces.action import Waypoint

import numpy as np
from sklearn.decomposition import PCA 
from copy import deepcopy
from time import sleep

def pose_dist(p1, p2):
  return ((p1.x - p2.x)**2 + (p1.y - p2.y)**2)**0.5

def get_pose_angle(p0, p1, p2):
    a10 = np.arctan2(p0.y - p1.y, p0.x- p1.x)
    a12 = np.arctan2(p2.y - p1.y, p2.x- p1.x)
    return a12 - a10 - np.pi

def fill_grid(grid):
  """
  Fill in a grid within a perimeter. For every point within the zone, driving in any
  cardinal direction will eventually lead you to a perimeter point, but points outside
  the zone will lead you to the edge of the grid.
  """
  directions = [[0,1], [0,-1], [1,0], [-1,0]]
  for (x, y), g in np.ndenumerate(grid):
    if g:
      continue
    inside = True
    for d in directions:
      x2 = x + d[0]
      y2 = y + d[1]
      perimeter_hit = False
      while x2 >= 0 and y2 >= 0 and x2 < grid.shape[0] and y2 < grid.shape[1]:
        if grid[x2,y2]:
          perimeter_hit = True
          break
        x2 += d[0]
        y2 += d[1]
      if not perimeter_hit:
        inside = False
        break
    if inside:
      grid[x,y] = 1
class Stripe():
  """
  Container for a single stripe within a zone. It contains two points representing 
  the beginning and end of a stripe. p1 will always be left of p2 in the coordinate
  system defined after the PCA transform.
  """
  def __init__(self, p1):
    self.p1 = p1

  def set_p2(self, p2):
    self.p2 = p2

  def dist(self, p):
    """
    returns the minimum distance from an endpoint to the point passed in: p
    """
    d1 = ((p[0] - self.p1[0])**2 + (p[1] - self.p1[1])**2) ** 0.5
    d2 = ((p[0] - self.p2[0])**2 + (p[1] - self.p2[1])**2) ** 0.5
    if d1 < d2:
      return d1, True
    #else
    return d2, False

def get_closest_stripe(stripes, p):
  #TODO this may need to use A* to find the stripe that has the shortest path as opposed 
  # to the stripe with the closest euclidean distance. This is in case the area between
  # two stripes can't be traversed
  min_dist = np.inf
  closest_stripe = None
  left_to_right = None
  for s in stripes:
    dist, ltr = s.dist(p)
    if dist < min_dist:
      min_dist = dist
      closest_stripe = s
      left_to_right = ltr
  return closest_stripe, left_to_right

def get_lowest_stripe(stripes):
  min_y = np.inf
  lowest_stripe = None
  for s in stripes:
    if s.p1[1] < min_y:
      min_y = s.p1[1]
      lowest_stripe = s
  return lowest_stripe

class Zone():
  def __init__(self):
    self.perimeter = []
    self.start_departed = False

  @classmethod
  def from_perimeter(cls, perimeter):
    zone = cls()
    zone.perimeter = perimeter
    return zone

  def add_perimeter(self, pose):
    #If we haven't left the start position, then append this pose
    if not self.start_departed: 
      self.perimeter.append(pose)
      if pose_dist(self.perimeter[0].pose.position, pose.pose.position) > 0.2:
        self.start_departed = True
      return True 
    # If we have left the start, but now we are back, then close the loop and finish this zone
    elif pose_dist(self.perimeter[0].pose.position, pose.pose.position) < 0.1:
      return False
    # If we have left the start, but haven't returned yet, then append this pose
    self.perimeter.append(pose)
    return True

  def get_path_message(self):
    msg = Path()
    msg.header = self.perimeter[-1].header
    msg.poses = self.perimeter
    return msg

  def thin_perimeter(self):
    """
    Remove points along the perimeter that are within 10cm of each other.
    """
    new_perimeter = [self.perimeter[0]]
    for pose in self.perimeter[1:]:
      if pose_dist(new_perimeter[-1].pose.position, pose.pose.position) < 0.1:
        continue
      new_perimeter.append(pose)
    self.perimeter = new_perimeter

  def complete_perimeter(self):
    """
    Connects the end points of the perimeter if they are farther than 10cm away
    """
    p0 = self.perimeter[0].pose.position
    while pose_dist(p0, self.perimeter[1].pose.position) > 0.1:
      new_pose = deepcopy(self.perimeter[1])
      p1 = new_pose.pose.position
      angle = np.arctan2(p0.y - p1.y, p0.x - p1.x)
      new_pose.pose.position.x += 0.1 * np.cos(angle)
      new_pose.pose.position.y += 0.1 * np.sin(angle)
      self.perimeter.append(new_pose)

  def is_clockwise(self):
    """
    If the perimeter was recorded in a clockwise fashion, then the sum of the turn angles at
    every point in the perimeter should be negative since there will be more left turns than right
    """
    turn_sum = 0
    for i, p in enumerate(self.perimeter):
      p_prev = self.perimeter[i - 1]
      p_next = self.perimeter[(i + 1) % len(self.perimeter)]
      turn_sum += get_pose_angle(p_prev.pose.position, p.pose.position, p_next.pose.position)
    return turn_sum > 0

  def coordinate_extrema(self):
    min_x, min_y, max_x, max_y = np.inf, np.inf, -np.inf, -np.inf
    for p in self.perimeter:
      min_x = min(p.pose.position.x, min_x)
      min_y = min(p.pose.position.y, min_y)
      max_x = max(p.pose.position.x, max_x)
      max_y = max(p.pose.position.y, max_y)
    return min_x, min_y, max_x, max_y

  def make_grid(self):
    """
    Make a grid with axes aligned to the map coordinate frame
    """
    self.resolution = 0.15 #TODO make this configurable
    self.min_x, self.min_y, self.max_x, self.max_y = self.coordinate_extrema()
    x_dim = int((self.max_x - self.min_x) / self.resolution) + 2
    y_dim = int((self.max_y - self.min_y) / self.resolution) + 2
    self.grid = np.zeros((x_dim, y_dim))

    # first mark all the grid cells along the perimeter
    for p in self.perimeter:
      x = int(np.round((p.pose.position.x - self.min_x) / self.resolution))
      y = int(np.round((p.pose.position.y - self.min_y) / self.resolution))
      self.grid[x, y] = 1

    # Then fill in the zone
    fill_grid(self.grid)

  def make_pca_perimeter(self):
    """
    Remake the grid where the axes are the principal components
    """
    # From the axis-aligned grid, calculate the principal components and transform so that
    # the axes point in the direction of the greatest variation (i.e. the longest stripes)
    X = np.squeeze(np.dstack(np.where(self.grid)))
    self.pca = PCA()
    self.pca.fit(X)

    # transform the perimeter to the PCA axes
    perimeter = []
    for p in self.perimeter:
      x = (p.pose.position.x - self.min_x) / self.resolution
      y = (p.pose.position.y - self.min_y) / self.resolution
      perimeter.append([x, y])
    perimeter = np.array(perimeter)
    perimeter_transform = self.pca.transform(perimeter)
    self.min_trans_x, self.min_trans_y = np.min(perimeter_transform, axis=0).astype(int)
    self.max_trans_x, self.max_trans_y = np.max(perimeter_transform, axis=0).astype(int)
    x_trans_dim = self.max_trans_x - self.min_trans_x + 2
    y_trans_dim = self.max_trans_y - self.min_trans_y + 2
    self.grid_transform = np.zeros((x_trans_dim, y_trans_dim))
    #first populate the transformed perimeter
    for x, y in perimeter_transform:
      x = int(np.round(x - self.min_trans_x))
      y = int(np.round(y - self.min_trans_y))
      self.grid_transform[x, y] = 1
    #then fill in any gaps that may exist
    last_coord = np.round(perimeter_transform[-1] - np.array([self.min_trans_x, self.min_trans_y])).astype(int)
    for x, y in perimeter_transform:
      x = int(np.round(x - self.min_trans_x))
      y = int(np.round(y - self.min_trans_y))
      x_diff = x - last_coord[0]
      y_diff = y - last_coord[1]
      dist = (x_diff**2 + y_diff**2) ** 0.5
      if dist >= 1.9:
        for i in range(int(dist)):
          last_coord[0] += (i + 1)/(int(dist) + 1) * x_diff
          last_coord[1] += (i + 1)/(int(dist) + 1) * y_diff
          self.grid_transform[np.round(last_coord).astype(int)] = 1

      last_coord = np.array([x, y])

  def get_waypoints(self):
    """
    Make a plan to canvas the entire zone and return the waypoints
    """
    self.make_grid()
    self.make_pca_perimeter()
    # first find all of the perimeter points that you can move right or left on
    canvas = self.grid_transform.copy()
    fill_grid(canvas)
    #we have already mowed the perimeter, so just mow the interior
    canvas[np.where(self.grid_transform)] = 0 
    x_dim = self.grid_transform.shape[0]
    stripes = []
    for x, y in zip(*np.where(self.grid_transform)):
      x2 = x + 1
      if x2 < x_dim and canvas[x2, y]:
        stripe = Stripe((x2, y))
        stripes.append(stripe)
        try:
          while canvas[x2, y]:
            stripe.set_p2((x2, y))
            x2 += 1
        except:
          breakpoint()
    cur_pos = get_lowest_stripe(stripes).p1
    waypoints = []
    while len(stripes):
      s, ltr = get_closest_stripe(stripes, cur_pos)
      if ltr:
        waypoints.append(self.inverse_transform(s.p1))
        waypoints.append(self.inverse_transform(s.p2))
        cur_pos = s.p2
      else:
        waypoints.append(self.inverse_transform(s.p2))
        waypoints.append(self.inverse_transform(s.p1))
        cur_pos = s.p1
      stripes.remove(s)
    return waypoints

  def inverse_transform(self, p):
    x, y = p
    #first transform from the pca coordinate system back into the axis aligned coordinate system
    x += self.min_trans_x
    y += self.min_trans_y
    x, y = self.pca.inverse_transform(np.array([x,y]))

    # then transform back to world coordinates
    x *= self.resolution
    y *= self.resolution
    x += self.min_x
    y += self.min_y
    return (x, y)

  def __len__(self):
    return len(self.perimeter)

class PathPlanner(Node):
  def __init__(self):
    super().__init__("path_planner")
    self.path_sub_ = self.create_subscription(
      Path
      , "/lawn_mower/zones"
      , self.process_zone
      , 10
    )

    self.perimeter_mapped_sub_ = self.create_subscription(
      Empty
      , "/lawn_mower/perimeter_mapped"
      , self.process_perimeter_mapped
      , 1
    )

    self.waypoint_marker_pub_ = self.create_publisher(
      MarkerArray
      , "/lawn_mower/waypoint_markers"
      , 10
    )

    self.waypoint_client_ = ActionClient(self, Waypoint, 'waypoint')
    self.zones = {}

  def process_zone(self, msg):
    zone_id = msg.header.stamp.sec
    if zone_id not in self.zones:
      self.get_logger().info("Adding new zone with id: {}".format(zone_id))
      self.zones[zone_id] = Zone.from_perimeter(msg.poses)
      self.zones[zone_id].complete_perimeter()

  def process_perimeter_mapped(self, msg):
    for zone_id, zone in self.zones.items():
      self.get_logger().info("Creating path for zone with id: {}".format(zone_id))
      self.waypoints = zone.get_waypoints()
      self.publish_waypoint_markers(self.waypoints, zone_id)
      self.send_waypoints()

  def send_waypoints(self):
    waypoint_msg = Waypoint.Goal()
    for w in self.waypoints:
      pt = Point()
      pt.x, pt.y = w
      waypoint_msg.waypoints.append(pt)

    self.waypoint_client_.wait_for_server()
    self._send_waypoint_future = self.waypoint_client_.send_goal_async(
                                  waypoint_msg
                                  , feedback_callback=self.feedback_callback
                                  )
    self._send_waypoint_future.add_done_callback(self.goal_response_callback)

  def goal_response_callback(self, future):
    goal_handle = future.result()
    if not goal_handle.accepted:
      self.get_logger().info("Goal rejected :(")
      return
    self.get_logger().info("Goal accepted :)")

    self._get_result_future = goal_handle.get_result_async()
    self._get_result_future.add_done_callback(self.get_result_callback)

  def get_result_callback(self, future):
    result = future.result().result
    self.get_logger().info("Result: success={}".format(result.success))

  def feedback_callback(self, feedback_msg):
    feedback = feedback_msg.feedback
    self.get_logger().info("Received feedback distance to waypoint is {:.2f}".format(feedback.distance))

  def publish_waypoint_markers(self, waypoints, zone_id):
    marker_array = MarkerArray()
    for i, wp in enumerate(waypoints[:-1]):
      marker = Marker()
      marker.type = Marker.ARROW
      marker.action = Marker.ADD
      marker.scale.x = 0.01
      marker.scale.y = 0.01
      marker.scale.z = 0.01
      marker.pose.position.x = 0.0
      marker.pose.position.y = 0.0
      marker.pose.position.z = 0.0
      marker.pose.orientation.x = 0.0
      marker.pose.orientation.y = 0.0
      marker.pose.orientation.z = 0.0
      marker.pose.orientation.w = 1.0
      marker.color.r = 0.0
      marker.color.g = i / len(waypoints)
      marker.color.b = 0.0
      marker.color.a = 1.0

      p1 = Point()
      p1.x = wp[0]
      p1.y = wp[1]
      marker.points.append(p1)
      p2 = Point()
      p2.x = waypoints[i+1][0]
      p2.y = waypoints[i+1][1]
      marker.points.append(p2)
      marker.id = zone_id + i
      marker.header.stamp = self.get_clock().now().to_msg()
      marker.header.frame_id = '/map'
      marker_array.markers.append(marker)
    self.waypoint_marker_pub_.publish(marker_array)


def main(args=None):
  rclpy.init(args=args)

  path_planner = PathPlanner()

  rclpy.spin(path_planner)

  # Destroy the node explicitly
  # (optional - otherwise it will be done automatically
  # when the garbage collector destroys the node object)
  path_planner.destroy_node()
  rclpy.shutdown()

if __name__ == '__main__':
  main()