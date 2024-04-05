import os
from datetime import datetime

class SearchDirectory():
  def __init__(self, root='rosbag'):
    self.default_dir = os.path.join(os.path.expanduser('~'), root)
    if not os.path.exists(self.default_dir):
      os.makedirs(self.default_dir)
      print(f"Directory '{self.default_dir}' created.\nSave your rosbags here!")
    
    self.folders_dir = []
    return
  
  def check_bags(self, dir):
    subdirectories = [d for d in os.listdir(dir) if os.path.isdir(os.path.join(dir, d))]
    if not subdirectories:
      return False
    else:
      for subs in subdirectories:
        check = os.path.join(dir,subs,subs+"_0.db3")
        if os.path.isfile(check):
          return True
      return False

  def search_offset(self):
    """
    when offset value is given, directory search will go in twice
    plot value of title will be set with offset
    return directories to search in list
    """
    # List subdirectories in the default_dir
    subdirectories = [d for d in os.listdir(self.default_dir) if os.path.isdir(os.path.join(self.default_dir, d))]
    for i, subdirectory in enumerate(subdirectories, 1):
      print(f"{i}. {subdirectory}")
    
    result_dirs=[]

    if subdirectories:
      user_choice = input("Enter the number of the folder you want to look for (-1, exit): ")
      try:
        user_choice = int(user_choice)
        if 1 <= user_choice <= len(subdirectories):
          folder = subdirectories[user_choice-1]
          print(f"{folder} is chosen")
          select_dir = os.path.join(self.default_dir, folder)
          user_dirs = [d for d in os.listdir(select_dir) if os.path.isdir(os.path.join(select_dir, d))]
          for i, dir in enumerate(user_dirs, 1):
            print(f"Plotting for {dir}")
            plot_dir = os.path.join(select_dir, dir, dir+'_0.db3')
            result_dirs.append(plot_dir)

        elif user_choice == -1:
          print("Exiting...")
        else: 
          print("Invalid choice. Exiting process...")
      except ValueError:
        print("Invalid input. Exiting process...")
    else:
      print("Nothing to search! Exiting...")
    
    return result_dirs
  
  def search(self):
    """
    directory search will go in once
    return directories to search in list
    """
    # List subdirectories in the default_dir
    subdirectories = [d for d in os.listdir(self.default_dir) if os.path.isdir(os.path.join(self.default_dir, d))]
    for i, subdirectory in enumerate(subdirectories, 1):
      print(f"{i}. {subdirectory}")
    
    result_dirs=[]

    if subdirectories:
      user_choice = input("Enter the number of the folder you want to look for (-1, exit / a for all): ")
      try:
        if user_choice == 'a':
          print("selecting for all!")
          for sub in subdirectories:
            result_dirs.append(os.path.join(self.default_dir, sub, sub+'_0.db3'))
            self.folders_dir.append(os.path.join(self.default_dir, sub))
        else:
          user_choice = int(user_choice)
          if 1 <= user_choice <= len(subdirectories):
            folder = subdirectories[user_choice-1]
            print(f"{folder} is chosen")
            result_dirs.append(os.path.join(self.default_dir, folder, folder+'_0.db3'))
            self.folders_dir.append(os.path.join(self.default_dir, folder))
          elif user_choice == -1:
            print("Exiting...")
          else: 
            print("Invalid choice. Exiting process...")
      except ValueError:
        print("Invalid input. Exiting process...")
    else:
      print("Nothing to search! Exiting...")
    
    return result_dirs
  
  def save_bag_dir(self, count: int, folder_name: str=None)->list[str]:
    """
    @brief saving your ros2 bag with today date and cnt number
    @count number of ros2 bags you want to save
    """
    today_date = datetime.now().date()
    formatted_date = today_date.strftime('%Y-%m-%d')
    save_dir = self.default_dir
    
    if folder_name:
      save_dir = os.path.join(save_dir, folder_name)
      if not os.path.exists(save_dir):
        os.makedirs(save_dir)
        print(f"Directory '{save_dir}' created.")

    
    # check existings dir
    subdirectories = [d for d in os.listdir(save_dir) if os.path.isdir(os.path.join(save_dir, d))]

    start_count = 0
    result_dir = []

    # check number start
    if subdirectories:
      numbers = [int(n[-2:]) for n in subdirectories]
      numbers.sort()
      # check maximum number
      if numbers[-1] >= start_count:
        start_count = numbers[-1]+1
    
    # saving dir
    for i in range(start_count, start_count+count):
      val = ''
      if i < 10:
        val = '0'+str(i)
      else:
        val = str(i)
      result_dir.append(os.path.join(save_dir, formatted_date+'-'+val))
    return result_dir