from ros2_data_plot.bag_parser.parser import Bag2FileParser
from ros2_data_plot.util.search_dir import SearchDirectory

def main():
  # get directory list
  searcher = SearchDirectory('rosbag')
  user_dir = searcher.search()
  for dir in user_dir:
    parser = Bag2FileParser(dir)
    # parser.show_speed("/cmd_vel", "/robot/odom")
    parser.show_spin("/cmd_vel", "/odom")

  print("successfully finished!")

if __name__ == '__main__':
  main()