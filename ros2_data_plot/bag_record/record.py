import rclpy
from rclpy.node import Node
from rclpy.serialization import serialize_message
from std_msgs.msg import String
import rosbag2_py
from rosbags.typesys import Stores, get_typestore
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import Twist, PoseWithCovarianceStamped
from std_msgs.msg import Bool

# https://docs.ros.org/en/rolling/Tutorials/Advanced/Recording-A-Bag-From-Your-Own-Node-Py.html#write-the-python-node
class BagRecorder(Node):
  def __init__(self, default_dir: str, topics: dict, end_topic: tuple)-> None: 
    """
    @default_dir what folder directory you want to save 'dir + folder name' 
    @topics {topic name, type} = {'/string', 'std_msgs/msg/String'}
    @end_topic (topic name, type) = ('/string', 'std_msgs/msg/String')
    """
    super().__init__('single_bag_recorder')
    self.writer = rosbag2_py.SequentialWriter()

    self.cbs, self.subs = [], []
    storage_options = rosbag2_py._storage.StorageOptions(
        uri=default_dir,
        storage_id='sqlite3')
    converter_options = rosbag2_py._storage.ConverterOptions('', '')
    self.writer.open(storage_options, converter_options)
    self.create_writers(topics)

    # recording end signal
    # pkg_name, pkg_msg, msg_type = end_topic[1].split('.')
    # md = importlib.import_module(pkg_name+'.'+pkg_msg)
    self.end_topic_sub = self.create_subscription(
                          self._get_msg_type(end_topic[1]),
                          end_topic[0],
                          self._callback_terminate_signal,
                          10)
    self.terminate_signal = False
    print("[INITIALIZE] subscription finished")

  def _get_msg_type(self, type_of:String):
    """
    @type_of 'pkg_name/pkg_msg/msg_type' (ex,'geometry_msgs/msg/Twist' )
    """
    print(type_of)
    if type_of == "geometry_msgs/msg/Twist":
      print("Twist")
      return Twist
    elif type_of == "geometry_msgs/msg/PoseWithCovarianceStamped":
      print("PoseWithCovarianceStamped")
      return PoseWithCovarianceStamped
    elif type_of == "nav_msgs/msg/Odometry":
      print("Odom")
      return Odometry
    elif type_of == "nav_msgs/msg/Path":
      print("Path")
      return Path
    elif type_of == "std_msgs/msg/Bool":
      print("Bool")
      return Bool
    else:
      print("None Type")
      return None
       
  def _callback_terminate_signal(self, msg):
    print("Terminate signal callback!")
    self.terminate_signal = True
    return
  
  def _create_callback(self, topic):
        def callback(msg):
            self.writer.write(
                topic,
                serialize_message(msg),
                self.get_clock().now().nanoseconds)
        return callback
  
  def create_writers(self, topics: dict):
    for topic in topics:
      topic_info = rosbag2_py._storage.TopicMetadata(
                  name=topic,
                  type=topics[topic],
                  serialization_format='cdr')
      self.writer.create_topic(topic_info)
      self.cbs.append(self._create_callback(topic))
      self.subs.append(self.create_subscription(
                      self._get_msg_type(topics[topic]),
                      topic,
                      self.cbs[-1],
                      10))
    return
  
  def run_node(self):
      while rclpy.ok() and not self.terminate_signal:
        rclpy.spin_once(self)

def main(args=None):
  rclpy.init(args=args)
  test = BagRecorder('/home/hd/testing', {'/odom':'nav_msgs/msg/Odometry'}, {'/NAV/Stop':'std_msgs/msg/Bool'})

if __name__ == '__main__':
  main()