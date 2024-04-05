from nav_msgs.msg import Path
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped, Pose, PoseWithCovariance, Twist, PoseWithCovarianceStamped
import numpy as np
from math import sqrt, pow

class ObjectType():
  def __init__(self, data=None):
    """
    @data [(timestamp0, message0), (timestamp1, message1), ...]
    """
    self.org = data # list[tuple[any, any], ...]
    if self.org:
      self.timestamps = [t[0] for t in self.org]
      self.data = [t[1] for t in self.org]
      print("data initialized!")
    else:
      print("data none initialized")
  
  def get_paths(self):
    """
    initialize format nav_msgs.msg PoseStamped
    return [x1, x2, ...] [y1, y2, ...] [deg1, deg2, ...]
    """
    return 
  
  def get_poses(self):
    """
    initialize format [(t1, Odometry1), (t2, Odometry2), ...]
    return [x1, x2, ...] [y1, y2, ...] [deg1, deg2, ...]
    """
    xs, ys, degs = [], [], []

    if isinstance(self.data[0], Odometry) or isinstance(self.data[0], PoseWithCovarianceStamped):
      for pose in self.data:
        x, y, deg = self._get_2dpose(pose)
        xs.append(x)
        ys.append(y)
        degs.append(deg)
    else: 
      print("wrong access of Odometry!")

    return xs, ys, degs
    
  def get_speeds(self):
    """
    initialize format [(t1, Odometry/Twist1), (t2, Odometry/Twist2), ...]
    return [vx1, vx2, ...] [vy1, vy2, ...] [vdeg1, vdeg2, ...]
    """
    vxs, vys, vdegs = [], [], []
    if isinstance(self.data[0], Odometry):
      for pose in self.data:
        vx, vy, vdeg = self._get_speeds(pose)
        vxs.append(vx)
        vys.append(vy)
        vdegs.append(vdeg)
    elif isinstance(self.data[0], Twist):
      for pose in self.data:
        vxs.append(pose.linear.x)
        vys.append(pose.linear.y)
        vdegs.append(pose.angular.z*180/np.pi)
    else:
      print("wrong access of Odometry or Twist!")
    return vxs, vys, vdegs
  
  def get_lin_speed(self):
    """
    initialize format [(t1, Odometry1), (t2, Odometry2), ...]
    return [v1, v2, ...] [m/s] vector
    """
    if isinstance(self.data[0], Odometry):
      vs = []
      for pose in self.data:
        vx, vy, vdeg = self._get_speeds(pose)
        val = sqrt(pow(vx,2)+pow(vy,2))
        vs.append(val)
      return vs
    elif isinstance(self.data[0], Twist):
      vs = []
      for d in self.data:
        vx = d.linear.x
        vy = d.linear.y
        val = sqrt(pow(vx,2)+pow(vy,2))
        vs.append(val)
      return vs
    else:
      print("wrong access of Odometry or Twist!")
      return []
    
  def get_ang_speed(self):
    """
    return [w1, w2, ...] [deg/s] vector
    """
    ws = []
    if isinstance(self.data[0], Odometry):
      for pose in self.data:
        vx, vy, vdeg = self._get_speeds(pose)
        ws.append(vdeg)
      return ws
    elif isinstance(self.data[0], Twist):
      for d in self.data:
        val = d.angular.z*180/np.pi
        ws.append(val)
      return ws
    else:
      print("wrong access of Odometry or Twist!")
      return []
    
  def get_accs(self, time, value):
    """
    @time [t1, t2]
    @value [speed1, speed2]
    @return [m/ss] or [deg/ss]
    """
    result = []
    for i in range(len(value)):
      if i==len(value)-1:
        break
      val = (value[i+1]-value[i])/(time[i+1]-time[i])
      result.append(val)
    return result
  
  def _get_2dpose(self, odom):
    x, y, deg = None, None, None
    if isinstance(odom, PoseStamped) or isinstance(odom, PoseWithCovariance):
      x = odom.pose.position.x
      y = odom.pose.position.y
      deg = np.arctan2(2.0 * (odom.pose.orientation.w * odom.pose.orientation.z + odom.pose.orientation.x * odom.pose.orientation.y),
                              1.0 - 2.0 * (odom.pose.orientation.y**2 + odom.pose.orientation.z**2))
    elif isinstance(odom, Pose):
      x = odom.position.x
      y = odom.position.y
      deg = np.arctan2(2.0 * (odom.orientation.w * odom.orientation.z + odom.orientation.x * odom.orientation.y),
                              1.0 - 2.0 * (odom.orientation.y**2 + odom.orientation.z**2))
    elif isinstance(odom, Odometry) or isinstance(odom, PoseWithCovarianceStamped):
      x = odom.pose.pose.position.x
      y = odom.pose.pose.position.y
      deg = np.arctan2(2.0 * (odom.pose.pose.orientation.w * odom.pose.pose.orientation.z + odom.pose.pose.orientation.x * odom.pose.pose.orientation.y),
                              1.0 - 2.0 * (odom.pose.pose.orientation.y**2 + odom.pose.pose.orientation.z**2))
    else:
      print("error getting 2d pose")

    return x, y, deg
  
  def _get_speeds(self, odom):
    """
    @odom Odometry
    """
    vx = odom.twist.twist.linear.x
    vy = odom.twist.twist.linear.y
    vdeg = odom.twist.twist.angular.z*180/np.pi
    return vx, vy, vdeg

  def get_timestamps(self):
    return self.timestamps
  
  def extract_poses(self, data):
    """
    external usage function
    @data Path
    """
    if not isinstance(data, Path):
      print("wrong access of Path!")
      return
    
    xs, ys, degs = [], [], []
    for pose in data.poses:
      x, y, deg = self._get_2dpose(pose)
      xs.append(x)
      ys.append(y)
      degs.append(deg)
    return xs, ys, degs