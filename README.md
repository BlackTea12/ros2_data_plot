# ros2_data_plot package

## plot_robot_vel

- Data to observe: target velocity, acutal robot velocity
- Result plot: Velocity, each wheel velocity
  
## plot_travel_data

- Data to observe: planned path, actual robot footprints, actual robot velocity
- Result plot: Trajectory, path tracking error, robot velocity
  
- Planned Path
  - /path
  
- acutal robot footprints and velocity
  - /robot/odom

To synchronize recorded data timesteps, robot timestep will be our standard since it is recorded more.
Based on our standard, we choose matching timestamp in /plan within certain time difference error (0.15secs).
With the selected data, we will compare path tracking error.

## plot_travel_data_precise

- Data to observe: planned path, actual robot localized points, actual robot velocity
- Result plot: Trajectory, path tracking error, robot velocity linear and angular comparison
  
- Planned Path
  - /path
  
- acutal robot footprints and velocity
  - /amcl_pose, /odom

### how to run

    ros2 run ros2_data_plot {python execution file name}