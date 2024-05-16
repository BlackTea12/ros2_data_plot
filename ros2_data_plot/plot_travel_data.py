from ros2_data_plot.bag_parser.parser import Bag2FileParser
from ros2_data_plot.util.search_dir import SearchDirectory
import os

def main():
  root = os.path.expanduser('~')
  folder = input(f'searching ros2 bag in {root}/? : ')

  # get directory list
  searcher = SearchDirectory(folder)
  if not searcher.check_bags(os.path.join(root,folder)):
    print("no ros2 bag file exists...")

  user_dir = searcher.search()

  # searcher = SearchDirectory('field_rosbag/rosbags_0313')
  # user_dir = searcher.search()
  # user_offset = input("Enter the offset value of the folder you selected: ")
  for dir in user_dir:
    print(f"showing plot for {dir}")
    parser = Bag2FileParser(dir)
    parser.show_navigation_related_to_robot("/plan", "/amcl_pose")

  print("successfully finished!")

if __name__ == '__main__':
  main()