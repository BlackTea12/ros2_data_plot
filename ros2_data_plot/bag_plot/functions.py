from math import dist

def find_path_point(rb_x, rb_y, xs, ys)-> list: 
  '''
  finding path closest point compared to robot position
  @rb_x robot x position
  @rb_y robot y position
  @xs updated path x position
  @ys updated path y position
  '''
  pose = [0,0]  # x, y
  min_dist = 1000
  for i in range(len(xs)):
    d = dist([rb_x, rb_y],[xs[i], ys[i]])
    if d < min_dist:
      min_dist = d
      pose[0] = xs[i]
      pose[1] = ys[i]
  return pose