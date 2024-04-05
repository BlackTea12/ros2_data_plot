from ros2_data_plot.bag_parser.parser import Bag2FileParser
from ros2_data_plot.util.search_dir import SearchDirectory

def main():
  # get directory list
  searcher = SearchDirectory('field_rosbag/rosbags_0313')
  user_dir = searcher.search()
  user_offset = input("Enter the offset value of the folder you selected: ")
  for dir in user_dir:
    parser = Bag2FileParser(dir)
    parser.show_navigation("/plan", "/robot/odom", user_offset)

  print("successfully finished!")

if __name__ == '__main__':
  main()