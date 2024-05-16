import sqlite3
from rosidl_runtime_py.utilities import get_message
from rclpy.serialization import deserialize_message
from nav_msgs.msg import Path
from nav_msgs.msg import Odometry
from ros2_data_plot.bag_parser.object import ObjectType as obtype
from ros2_data_plot.bag_plot.plot import DrawPlot as drawer
from ros2_data_plot.bag_plot.functions import find_path_point

class Bag2FileParser():
  def __init__(self, bag_file):
    print(f"data for: {bag_file}")
    try:
      self.conn = sqlite3.connect(bag_file)
      self.cursor = self.conn.cursor()
      ## create a message type map
      topics_data = self.cursor.execute("SELECT id, name, type FROM topics").fetchall()

      # {'/plan': 'nav_msgs/msg/Path', '/robot/odom': 'nav_msgs/msg/Odometry'}
      self.topic_type = {name_of:type_of for id_of, name_of,type_of in topics_data}

      # {'/plan': 1, '/robot/odom': 2}
      self.topic_id = {name_of:id_of for id_of, name_of, type_of in topics_data}

      # {'/plan': <class 'nav_msgs.msg._path.Path'>, '/robot/odom': <class 'nav_msgs.msg._odometry.Odometry'>}
      self.topic_msg_message = {name_of:get_message(type_of) for id_of, name_of, type_of in topics_data}

    except sqlite3.Error as e:
      print(e)
      return
    
  def __del__(self):
    try:
      self.conn.close()
    except sqlite3.Error as e:
      print(e)
      return

  def get_messages(self, topic_name):
    """
    Return [(timestamp0, message0), (timestamp1, message1), ...]
    """
    topic_id = self.topic_id[topic_name]
    rows = self.cursor.execute("SELECT timestamp, data FROM messages WHERE topic_id = {}".format(topic_id)).fetchall()
    return [ (timestamp,deserialize_message(data, self.topic_msg_message[topic_name])) for timestamp,data in rows]

  def show_navigation(self, topic_traj, topic_rb, offset=''):
    """
    @topic_traj Path
    @topic_rb Odometry
    topic type: Path
    1. Plot trajectory in x-y plane
    2. Plot actual robot odometry
    3. Plot postion error [m]
    """
    paths = self.get_messages(topic_traj)
    traj_time, traj_x, traj_y = [], [], []
    object_parser = obtype()
    for timestamp, path in paths:
      xs, ys, degs = object_parser.extract_poses(path)
      if len(xs):
        traj_time.append(timestamp)
        traj_x.append(xs[0])
        traj_y.append(ys[0])
    print("finished path extraction!")
    poses = self.get_messages(topic_rb)
    robot_parser = obtype(poses)
    rb_time = robot_parser.get_timestamps()
    rb_x, rb_y, rb_deg = robot_parser.get_poses()
    vx, vy, vdeg = robot_parser.get_speeds()
    print("finished robot extraction!")
    print(f"{len(vx)} and {len(vy)}")
    ploter = drawer(traj_x=traj_x, traj_y=traj_y, traj_time=traj_time,
                    rb_x=rb_x, rb_y=rb_y, rb_time=rb_time,
                    vx=vx , vy=vy)
    ploter.xyplane(offset)
    return

  def show_navigation_precise(self, topic_traj, topic_rb, topic_loc, topic_cmd):
    """
    @topic_traj Path
    @topic_rb Odom
    @topic_loc PoseWithCovarianceStamped
    @topic_cmd Twist
    Path is published only once. Check path lateral error during travel and speed following
    """
    path_msg = self.get_messages(topic_traj)
    traj_time, traj_x, traj_y = [], [], []
    object_parser = obtype(path_msg)
    for timestamp, path in path_msg:
      xs, ys, degs = object_parser.extract_poses(path)
      traj_time = timestamp
      traj_x = xs
      traj_y = ys

    odom_msg = self.get_messages(topic_rb)
    odom_parser = obtype(odom_msg)
    loc_msg = self.get_messages(topic_loc)
    loc_parser = obtype(loc_msg)
    cmd_msg = self.get_messages(topic_cmd)
    cmd_parser = obtype(cmd_msg)

    rb_time = odom_parser.get_timestamps()
    rb_vx, rb_vy, rb_wz = odom_parser.get_speeds()
    loc_x, loc_y, loc_deg = loc_parser.get_poses()
    cmd_vx, cmd_vy, cmd_wz = cmd_parser.get_speeds()

    ploter = drawer(traj_x=traj_x, traj_y=traj_y,
                    loc_x=loc_x, loc_y=loc_y, loc_time=loc_parser.get_timestamps(),
                    rb_vx=rb_vx, rb_vy=rb_vy, rb_wz=rb_wz, rb_time=odom_parser.get_timestamps(),
                    cmd_vx=cmd_vx, cmd_vy=cmd_vy, cmd_wz=cmd_wz, cmd_time=cmd_parser.get_timestamps())
    ploter.path_track_vel_cmp()
    return
  
  def show_navigation_related_to_robot(self, topic_traj, topic_rb, offset=''):
    """
    @topic_traj Path
    @topic_rb Odometry
    topic type: Path
    1. Plot trajectory in x-y plane
    2. Plot actual robot odometry
    3. Plot postion error [m]
    """
    paths = self.get_messages(topic_traj)
    traj_time, trajs_x, trajs_y = [], [], []
    object_parser = obtype()
    for timestamp, path in paths:
      xs, ys, degs = object_parser.extract_poses(path)
      if len(xs):
        traj_time.append(timestamp)
        trajs_x.append(xs)
        trajs_y.append(ys)
    print("finished path extraction!")

    poses = self.get_messages(topic_rb)
    robot_parser = obtype(poses)
    rb_time = robot_parser.get_timestamps()
    rb_x, rb_y, rb_deg = robot_parser.get_poses()
    vx, vy, vdeg = robot_parser.get_speeds()
    print("finished robot extraction!")

    # finding related path points based on robot
    traj_x = []
    traj_y = []
    traj_filtered_time = []
    for i in range(len(traj_time)):
      for j in range(len(rb_time)):
        if abs(traj_time[i]-rb_time[j]) < 500000000:
          result_point = find_path_point(rb_x[j], rb_y[j], trajs_x[i], trajs_y[i])
          traj_x.append(result_point[0])
          traj_y.append(result_point[1])
          traj_filtered_time.append(traj_time[i])
          break
    ploter = drawer(traj_x=traj_x, traj_y=traj_y, traj_time=traj_filtered_time,
                    rb_x=rb_x, rb_y=rb_y, rb_time=rb_time,
                    vx=vx , vy=vy)
    ploter.xyplane(offset)
    return

  def show_speed(self, topic_cmdv, topic_rb, b_acc_plot=False):
    """
    @topic_cmdv Twist
    @topic_rb Odom
    Plot corresponding timeline of velocity V[m/s]
    """
    cmd_vel = self.get_messages(topic_cmdv)
    robot = self.get_messages(topic_rb)

    cmdvel_parser = obtype(cmd_vel)
    cmdv_t = cmdvel_parser.get_timestamps()
    cmdv_v = cmdvel_parser.get_lin_speed()

    robot_parser = obtype(robot)
    rb_v = robot_parser.get_lin_speed()
    rb_t = robot_parser.get_timestamps()
    rb_acc = robot_parser.get_accs(rb_t, rb_v)

    # print(f"cmd: {len(cmdv_t)}, {len(cmdv_v)}\nrobot: {len(rb_t)}, {len(rb_v)}")
    
    ploter = drawer(cmd_v=cmdv_v, cmd_time=cmdv_t,
                    rb_v=rb_v, rb_time=rb_t, rb_acc=rb_acc)
    
    if not b_acc_plot:
      ploter.velocity_comp()
    else:
      ploter.vel_acc_comp()
    return
  
  def show_spin(self, topic_cmdv, topic_rb):
    """
    @topic_cmdv Twist
    @topic_rb Odom
    Plot corresponding timeline of velocity w[deg/s]
    """
    cmd_vel = self.get_messages(topic_cmdv)
    robot = self.get_messages(topic_rb)

    cmdvel_parser = obtype(cmd_vel)
    cmd_t = cmdvel_parser.get_timestamps()
    cmd_w = cmdvel_parser.get_ang_speed()

    robot_parser = obtype(robot)
    rb_w = robot_parser.get_ang_speed()
    rb_t = robot_parser.get_timestamps()

    
    ploter = drawer(cmd_w=cmd_w, cmd_time=cmd_t,
                    rb_w=rb_w, rb_time=rb_t)
    ploter.spin_comp()
    return