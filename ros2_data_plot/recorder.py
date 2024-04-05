import rclpy
from rclpy.node import Node
from ros2_data_plot.bag_record.record import BagRecorder
from ros2_data_plot.util.search_dir import SearchDirectory

def main(args=None):
  searcher = SearchDirectory('rosbag')
  user_save_dir = searcher.save_bag_dir(8, 'before_turtle')
  # user_save_dir = searcher.save_bag_dir(8, 'after_turtle')
  rclpy.init(args=args)

  topics = {'/odom':'nav_msgs/msg/Odometry', '/plan':'nav_msgs/msg/Path', '/cmd_vel':'geometry_msgs/msg/Twist', '/amcl_pose':'geometry_msgs/msg/PoseWithCovarianceStamped'}
  end_topic = ('/NAV/goal_reached','std_msgs/msg/Bool')
  for save_dir in user_save_dir:
    nd = BagRecorder(save_dir, topics, end_topic)
    try:
      nd.run_node()
    except Exception as e:
      nd.get_logger().error(f"An error occured: {str(e)}")
    finally:
      nd.destroy_node()
  
  rclpy.shutdown()

if __name__ == '__main__':
    main()