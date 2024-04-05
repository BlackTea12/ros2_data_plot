from ros2_data_plot.bag_parser.parser import Bag2FileParser
from ros2_data_plot.util.search_dir import SearchDirectory
from ros2_data_plot.util.meta_mod import MetaDataModify
import os

def main(args=None):
  root = os.path.expanduser('~')
  folder = input(f'searching ros2 bag in {root}/? : ')
  
  # get directory list
  searcher = SearchDirectory(folder)
  if not searcher.check_bags(os.path.join(root,folder)):
    print("no ros2 bag file exists...")

  user_dir = searcher.search()

  for dir in user_dir:
    parser = Bag2FileParser(dir)
    parser.show_navigation_precise("/plan", "/odom", "/amcl_pose", "/cmd_vel")

  print("successfully finished!")

if __name__ == '__main__':
  main()