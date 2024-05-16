import matplotlib.pyplot as plt
from math import ceil, sqrt, dist

SMALL_SIZE = 15
MEDIUM_SIZE = 18
BIGGER_SIZE = 22

class DrawPlot():
  def __init__(self, *args, **kwargs):
    '''
    drawing options
    * trajectory x-y plane [m]
    * time[sec] - position error[m]
    * actual trajectory with path start point [m]
    '''
    self.args = args
    self.kwargs = kwargs

  def xyplane(self, offset=''):
    traj_x = self.kwargs.get('traj_x', None)
    traj_y = self.kwargs.get('traj_y', None)
    # deg = self.kwargs.get('traj_deg', None)
    traj_t = self.kwargs.get('traj_time', None)
    if len(traj_x) == 0 or len(traj_y) == 0 or len(traj_t) == 0: 
      print("Length 0 exists in trajectory! Please check your data")

    rb_x = self.kwargs.get('rb_x', None)
    rb_y = self.kwargs.get('rb_y', None)
    # rb_deg = self.kwargs.get('rb_deg', None)
    rb_t = self.kwargs.get('rb_time', None)
    if len(rb_x) == 0 or len(rb_y) == 0 or len(rb_t) == 0: 
      print("Length 0 exists in trajectory! Please check your data")

    # speed_x = self.kwargs.get('vx', None)
    # speed_y = self.kwargs.get('vy', None)

    # Create subplots
    # fig, (ax1, ax2, ax3) = plt.subplots(3, 1)
    fig, (ax1, ax2) = plt.subplots(2, 1)
    if offset:
      fig.suptitle("offset_from_furthest="+offset, weight='bold', fontsize=14)
    ax1.set_title("Trajectory Results", weight='bold', fontsize=12)
    ax1.plot(traj_x, traj_y, color='#9467bd', label='follow path')
    ax1.plot(rb_x, rb_y, '--', color='#ff6969', label='robot trajectory')
    ax1.set_xlabel('X-axis [m]')
    ax1.set_ylabel('Y-axis [m]')
    ax1.set_xlim([-1.5,1.5])
    ax1.legend(loc='best')
    # annotation note
    # point_rb = ceil(len(rb_x)*2/3)
    # point_tra = ceil(len(traj_x)*2/3)
    # an1 = ax1.annotate('robot', xy=(rb_x[point_rb], rb_y[point_rb]), 
    #                xycoords='data', xytext=(3, rb_y[point_rb]-0.3),
    #                arrowprops=dict(facecolor='black', shrink=0.01))
    # an2 = ax1.annotate('planned path', xy=(traj_x[point_tra], traj_y[point_tra]), 
    #                xycoords='data', xytext=(3, traj_y[point_tra]-0.3),
    #                arrowprops=dict(facecolor='black', shrink=0.01))
    # an1.draggable()
    # an2.draggable()
    error_data = self.posistion_error(rb_x, rb_y, rb_t, traj_x, traj_y, traj_t)
    time_error, errors = zip(*error_data)
    ax2.set_title("Path Tracking Error", weight='bold', fontsize=12)
    ax2.plot(time_error, errors, color='#ff6969', label='lateral error [m]')
    ax2.set_xlabel('[sec]')
    ax2.set_ylabel('[m]')
    ax2.grid(axis='y')
    ax2.legend(loc='best')

    # time_vel, vel = self.get_velocity(speed_x, speed_y, rb_t)
    # ax3.set_title("Robot Speed", weight='bold', fontsize=12)
    # ax3.plot(time_vel, vel, label='v [m/s]')
    # ax3.set_xlabel('[sec]')
    # ax3.set_ylabel('[m/s]')
    # ax3.grid(axis='y')
    # ax3.legend(loc='best')
    
    # Adjust the layout of the subplots
    plt.legend(loc='best')
    fig.tight_layout()
    plt.show()
    return
  
  def velocity_comp(self):
    rb_v = self.kwargs.get('rb_v', None)
    rb_t = self.kwargs.get('rb_time', None)
    cmdv_v = self.kwargs.get('cmd_v', None)
    cmdv_t = self.kwargs.get('cmd_time', None)

    # choose smaller start time
    start_time = min(rb_t[0], cmdv_t[0])
    rb_t = [(t-start_time)*1e-9 for t in rb_t]
    cmdv_t = [(t-start_time)*1e-9 for t in cmdv_t]

    # plot
    plt.title("Target velocity and robot velocity")
    plt.plot(rb_t, rb_v, label='/robot/odom')
    plt.plot(cmdv_t, cmdv_v, '--', label='/cmd_vel')
 
    plt.rc('font', size=SMALL_SIZE)          # controls default text sizes
    plt.rc('axes', titlesize=SMALL_SIZE)     # fontsize of the axes title
    plt.rc('axes', labelsize=MEDIUM_SIZE)    # fontsize of the x and y labels
    plt.rc('xtick', labelsize=SMALL_SIZE)    # fontsize of the tick labels
    plt.rc('ytick', labelsize=SMALL_SIZE)    # fontsize of the tick labels
    plt.rc('legend', fontsize=SMALL_SIZE)    # legend fontsize
    plt.rc('figure', titlesize=BIGGER_SIZE)  # fontsize of the figure title

    plt.xlabel('time [sec]', weight='bold')
    plt.ylabel('v [m/s]', weight='bold')
    plt.grid(axis='y')
    plt.legend(loc='best')
    plt.show()
    return
  
  def vel_acc_comp(self):
    rb_v = self.kwargs.get('rb_v', None)
    rb_t = self.kwargs.get('rb_time', None)
    rb_acc = self.kwargs.get('rb_acc', None)
    cmdv_v = self.kwargs.get('cmd_v', None)
    cmdv_t = self.kwargs.get('cmd_time', None)
    
    # choose smaller start time
    start_time = min(rb_t[0], cmdv_t[0])
    rb_t = [(t-start_time)*1e-9 for t in rb_t]
    cmdv_t = [(t-start_time)*1e-9 for t in cmdv_t]

    # plot
    # Create subplots
    fig, (ax1, ax2) = plt.subplots(1, 2)
    ax1.set_title("Target velocity and robot velocity", weight='bold', fontsize=14)
    ax1.plot(rb_t, rb_v, label='/robot/odom')
    ax1.plot(cmdv_t, cmdv_v, '--', label='/cmd_vel')

    ax1.set_xlabel('time [sec]', weight='bold', fontsize=12)
    ax1.set_ylabel('v [m/s]', weight='bold', fontsize=12)
    ax1.grid(axis='y')
    ax1.legend(loc='best')
    
    ax2.set_title("Robot acceleration", weight='bold', fontsize=14)
    ax2.plot(rb_t[0:-1], rb_acc)

    ax2.set_xlabel('time [sec]', weight='bold', fontsize=12)
    ax2.set_ylabel('[m/s^2]', weight='bold', fontsize=12)
    ax2.grid(axis='y')
    ax2.legend(loc='best')

    # Adjust the layout of the subplots
    plt.legend(loc='best')
    fig.tight_layout()
    plt.show()
    return
  
  def spin_comp(self):
    rb_w = self.kwargs.get('rb_w', None)
    rb_t = self.kwargs.get('rb_time', None)
    cmd_w = self.kwargs.get('cmd_w', None)
    cmd_t = self.kwargs.get('cmd_time', None)

    # choose smaller start time
    start_time = min(rb_t[0], cmd_t[0])
    rb_t = [(t-start_time)*1e-9 for t in rb_t]
    cmd_t = [(t-start_time)*1e-9 for t in cmd_t]

    # plot
    plt.title("Target angular velocity and robot angular velocity")
    plt.plot(rb_t, rb_w, label='/odom')
    plt.plot(cmd_t, cmd_w, '--', label='/cmd_vel')
 
    plt.rc('font', size=SMALL_SIZE)          # controls default text sizes
    plt.rc('axes', titlesize=SMALL_SIZE)     # fontsize of the axes title
    plt.rc('axes', labelsize=MEDIUM_SIZE)    # fontsize of the x and y labels
    plt.rc('xtick', labelsize=SMALL_SIZE)    # fontsize of the tick labels
    plt.rc('ytick', labelsize=SMALL_SIZE)    # fontsize of the tick labels
    plt.rc('legend', fontsize=SMALL_SIZE)    # legend fontsize
    plt.rc('figure', titlesize=BIGGER_SIZE)  # fontsize of the figure title

    plt.xlabel('time [sec]', weight='bold')
    plt.ylabel('w [deg/s]', weight='bold')
    plt.grid(axis='y')
    plt.legend(loc='best')
    plt.show()
    return
  
  def path_track_vel_cmp(self):
    traj_x = self.kwargs.get('traj_x', None)
    traj_y = self.kwargs.get('traj_y', None)
    
    # rb_x = self.kwargs.get('rb_x', None)
    # rb_y = self.kwargs.get('rb_y', None)
    # rb_t = self.kwargs.get('rb_time', None)

    rb_vx = self.kwargs.get('rb_vx', None)
    rb_vy = self.kwargs.get('rb_vy', None)
    rb_wz = self.kwargs.get('rb_wz', None)
    rb_t = self.kwargs.get('rb_time', None)

    cmd_vx = self.kwargs.get('cmd_vx', None)
    cmd_vy = self.kwargs.get('cmd_vy', None)
    cmd_wz = self.kwargs.get('cmd_wz', None)
    cmd_t = self.kwargs.get('cmd_time', None)

    loc_x = self.kwargs.get('loc_x', None)
    loc_y = self.kwargs.get('loc_y', None)
    loc_t = self.kwargs.get('loc_time', None)

    # choose smaller start time
    start_time = min(rb_t[0], cmd_t[0])
    rb_t = [(t-start_time)*1e-9 for t in rb_t]
    cmd_t = [(t-start_time)*1e-9 for t in cmd_t]

    # Create subplots
    fig, ((ax1, ax2), (ax3, ax4)) = plt.subplots(2, 2, figsize=(10,8))
    ax1.set_title("Trajectory Results", weight='bold', fontsize=12)
    ax1.plot(traj_x, traj_y, color='#9467bd', label='planned path')
    ax1.plot(loc_x, loc_y, '--', color='#ff6969', label='robot trajectory')
    ax1.set_xlabel('X-axis [m]')
    ax1.set_ylabel('Y-axis [m]')
    ax1.legend(loc='best')
    # annotation note
    # point_rb = ceil(len(rb_x)*2/3)
    # point_tra = ceil(len(traj_x)*2/3)
    # an1 = ax1.annotate('robot', xy=(rb_x[point_rb], rb_y[point_rb]), 
    #                xycoords='data', xytext=(3, rb_y[point_rb]-0.3),
    #                arrowprops=dict(facecolor='black', shrink=0.01))
    # an2 = ax1.annotate('planned path', xy=(traj_x[point_tra], traj_y[point_tra]), 
    #                xycoords='data', xytext=(3, traj_y[point_tra]-0.3),
    #                arrowprops=dict(facecolor='black', shrink=0.01))
    # an1.draggable()
    # an2.draggable()
    t_loc_sec = [(t-loc_t[0])*1e-9 for t in loc_t]
    error_data = self.lateral_error(traj_x, traj_y, loc_x, loc_y)
    ax2.set_title("Path Tracking Error", weight='bold', fontsize=12)
    ax2.plot(t_loc_sec, error_data, color='#ff6969', label='lateral error [m]')
    ax2.set_xlabel('[sec]')
    ax2.set_ylabel('[m]')
    ax2.grid()
    ax2.legend(loc='best')

    rb_vel = self.get_velocity(rb_vx, rb_vy)
    cmd_vel = self.get_velocity(cmd_vx, cmd_vy)
    ax3.set_title("Linear Speed Comparison", weight='bold', fontsize=12)
    ax3.plot(rb_t, rb_vel, label='robot [m/s]')
    ax3.plot(cmd_t, cmd_vel, '--', label='command [m/s]')
    ax3.set_xlabel('[sec]')
    ax3.set_ylabel('v')
    ax3.grid()
    ax3.legend(loc='best')
    ax4.set_title("Angular Speed Comparison", weight='bold', fontsize=12)
    ax4.plot(rb_t, rb_wz, label='robot [deg/s]')
    ax4.plot(cmd_t, cmd_wz, '--', label='command [deg/s]')
    ax4.set_xlabel('[sec]')
    ax4.set_ylabel('wz')
    ax4.grid()
    ax4.legend(loc='best')

    # Adjust the layout of the subplots
    plt.legend(loc='best')
    fig.tight_layout()
    plt.show()
    return
  
  def posistion_error(self, rb_x, rb_y, rb_time, traj_x, traj_y, traj_time):
    """
    /robot/odom usually have more timestamps.
    selection of time will be based on /plan
    Return [(timestamp1, error1), (timestamp2, error2), ...]
    """

    # start and endtime based on robot time
    t_rb_sec = [(t-rb_time[0])*1e-9 for t in rb_time]
    t_traj_sec = [(t-rb_time[0])*1e-9 for t in traj_time]
    end_time = min(ceil(t_rb_sec[-1]), ceil(t_traj_sec[-1])) - t_rb_sec[0]
    print(f'0 to {end_time} to search')

    rb_index = 0
    rb_comp, traj_comp = [], []

    # searching based on path index
    for idx in range(len(t_traj_sec)):
      # check for max time search
      if t_traj_sec[idx] > end_time:
        break
      for i in range(rb_index, len(t_rb_sec)):
        comp_val = abs(t_rb_sec[i] - t_traj_sec[idx])
        if comp_val < 0.15:
          rb_index = i
          rb_comp.append(i)
          traj_comp.append(idx)
          break
     
    # calculating position error [m]
    if len(rb_comp) != len(traj_comp):
      print("Error in comparing length!")
      return
    
    error=[]
    for i in range(len(rb_comp)):
      err_x = pow(rb_x[rb_comp[i]]-traj_x[traj_comp[i]],2)
      err_y = pow(rb_y[rb_comp[i]]-traj_y[traj_comp[i]],2)
      err = sqrt(pow(err_x,2)+pow(err_y,2))
      error.append(err)
    return [(t_rb_sec[rb_comp[i]], error[i]) for i in range (len(error))]

  def lateral_error(self, traj_x, traj_y, rb_x, rb_y):
    # path max resolution to 1cm
    RES = 0.01
    path_x, path_y = [], []
    for i in range(0, len(traj_x)-1):
      d = dist([traj_x[i], traj_y[i]],[traj_x[i+1], traj_y[i+1]])
      if d > RES:
        cnt = ceil(d/RES)
        c = (traj_y[i+1]-traj_y[i])/(traj_x[i+1]-traj_x[i])
        for j in range(cnt):
          path_x.append(traj_x[i]+RES*j)
          y = c*(traj_x[i]+RES*j-traj_x[i])+traj_y[i]
          path_y.append(y)
      else:
        path_x.append(traj_x[i])
        path_y.append(traj_y[i])
    
    # calculate start
    err = []
    least_err = 0.0009
    for i in range(len(rb_x)):
      min_comp = 1000
      for j in range(len(path_x)):
        d = dist([rb_x[i], rb_y[i]],[path_x[j], path_y[j]])
        if d < least_err:
          err.append(0)
          min_comp = -1
          break
        elif d < min_comp:
          min_comp = d
      if not min_comp == -1:
        err.append(min_comp)
      
    return err
  
  def get_velocity(self, vx, vy, timestamp=None):
    if timestamp:
      time_sec = [(t-timestamp[0])*1e-9 for t in timestamp]
      vel = [sqrt(pow(vx[i],2)+pow(vy[i],2)) for i in range(len(vx))]
      return time_sec, vel
    else:
      vel = [sqrt(pow(vx[i],2)+pow(vy[i],2)) for i in range(len(vx))]
      return vel
