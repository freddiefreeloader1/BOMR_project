import numpy as np
import matplotlib.pyplot as plt
class Quit(SystemExit):
    pass

class SharedData():
    robot = None
    thymio_angle = 0
    thymio_position = (0,0)
    metric_path = []
    path_shared = []
    end = (0,0)
    heading = 0

shared = SharedData()

def get_shared():
    global shared
    return shared
def set_shared(s):
    global shared
    shared.robot = s.robot
    shared.thymio_position = s.thymio_position
    shared.thymio_angle = s.thymio_angle
    shared.metric_path = s.metric_path
    shared.path_shared = s.path_shared
    shared.end = s.end
    shared.heading = s.heading

def convert_camera_to_robot(position = None, angle = None, path = None):
    if angle is not None: angle = -angle 
    if position is not None: 
        position = (position[0]/1000.0, -position[1]/1000.)
    if path is not None:
        path = np.array([(p[0]/1000.0, -p[1]/1000.) for p in path])
    
    return position, angle, path

def plot_data(vel_meas,acc_meas,spin_meas,kalman_history_vel,kalman_history_acce,kalman_history_spin,absolute_history,kalman_history,metric_path,
              absolute_orientation,kalman_history_orientation,path_shared):
    "Plot every data from the path , take the history of measurement and kalman for the path"
    #convert into array, create linspace for display
    meas_time = np.linspace(0,1,len(spin_meas))
    kalman_history_vel = np.array(kalman_history_vel)
    kalman_history_acce = np.array(kalman_history_acce)
    kalman_history_spin = np.array(kalman_history_spin)
    history_time = np.linspace(0,1,len(kalman_history_acce))

    absolute_history = np.array(absolute_history)
    kalman_history = np.array(kalman_history)
    metric_path = np.array(metric_path)
    absolute_orientation = np.array(absolute_orientation)
    kalman_history_orientation = np.array(kalman_history_orientation)
    path_display = np.array(shared.path_shared)

    #first plot
    #VelocityX
    plt.figure(figsize=(15, 10))
    plt.subplot(3,2,1)
    plt.plot(meas_time,vel_meas[:,0],label = 'Measured X velocity (wheel)')
    plt.plot(history_time,kalman_history_vel[:,0],label = 'Kalman X velocity')
    plt.title('Velocity comparison')
    plt.xlabel('Arbitrary time')
    plt.ylabel('Velocity (m/s)')
    plt.legend()

    #Acceleration x
    plt.subplot(3,2,2)
    meas_time_acc = np.linspace(0,1,len(acc_meas))
    plt.plot(meas_time_acc,acc_meas[:,0],label = 'Measured X acceleration (accelerometer)')
    plt.plot(history_time,kalman_history_acce[:,0],label = 'Kalman X acceleration')
    plt.title('Acceleration comparison')
    plt.xlabel('Arbitrary time')
    plt.ylabel('Acceleration (m/s^2)')
    plt.legend()

    #Acceleration Y
    plt.subplot(3,2,3)
    plt.plot(meas_time_acc,acc_meas[:,1],label = 'Measured Y acceleration (accelerometer)')
    plt.plot(history_time,kalman_history_acce[:,1],label = 'Kalman Y acceleration')
    plt.title('Acceleration comparison')
    plt.xlabel('Arbitrary time')
    plt.ylabel('Acceleration (m/s^2)')
    plt.legend()

    #Velocity Y
    plt.subplot(3,2,4)
    plt.plot(meas_time,vel_meas[:,1],label = 'Measured Y velocity (wheel)')
    plt.plot(history_time,kalman_history_vel[:,1],label = 'Kalman Y velocity')
    plt.title('Velocity comparison')
    plt.xlabel('Arbitrary time')
    plt.ylabel('Velocity (m/s)')
    plt.legend()

    #Spin
    plt.subplot(3,2,5)
    plt.plot(meas_time,spin_meas,label = 'Measured spin (wheel)')
    plt.plot(history_time,kalman_history_spin,label = 'Kalman spin')
    plt.title('Spin Comparison')
    plt.xlabel('Arbitrary time')
    plt.ylabel('Spin rad/s')
    plt.legend()

    #Heading
    plt.subplot(3,2,6)
    meas_time_camera = np.linspace(0,1,len(absolute_orientation))
    plt.plot(meas_time_camera,absolute_orientation,label = 'Measured heading (camera)')
    x_time = np.linspace(0,1,len(kalman_history_orientation))
    plt.plot(x_time,kalman_history_orientation - (2*np.pi),label = 'Kalman heading')
    plt.title('Heading Comparison')
    plt.xlabel('Arbitrary time')
    plt.ylabel('Heading [rad]')
    plt.legend()

    plt.tight_layout()
    plt.show()

    plt.figure(figsize=(15, 10))

    plt.scatter(absolute_history[:, 0], absolute_history[:, 1], label='Measured position (camera)', s=1)
    plt.scatter(kalman_history[:, 0], kalman_history[:, 1], label='Kalman position', s=1)

    plt.title('Position Comparison')
    plt.xlabel('Arbitrary time')
    plt.ylabel('position [m]')
    plt.legend()
    
    plt.tight_layout()
    plt.show()

    plt.figure(figsize=(15, 10))

    plt.subplot(2, 2, 1)
    plt.plot(absolute_history[:, 0], absolute_history[:, 1], label='Absolute Position')
    plt.scatter(metric_path[:, 0], metric_path[:, 1], color='red', label='Metric Path Points', marker='x')
    plt.scatter(path_display[:,0],path_display[:,1],label = 'Path from shared',marker = 'o')
    plt.title('Absolute Position History')
    plt.xlabel('X')
    plt.ylabel('Y')
    plt.legend()

    plt.subplot(2, 2, 2)
    plt.plot(kalman_history[:, 0], kalman_history[:, 1], label='Kalman Position')
    plt.title('Kalman Position History')
    plt.xlabel('X')
    plt.ylabel('Y')
    plt.legend()

    plt.subplot(2, 2, 3)
    plt.plot(kalman_history_orientation, label='Kalman Orientation')
    plt.title('Orientation History')
    plt.xlabel('Time Steps')
    plt.ylabel('Orientation')
    plt.legend()

    plt.subplot(2, 2, 4)
    plt.plot(absolute_orientation, label='Absolute Orientation')
    plt.title('Orientation History')
    plt.xlabel('Time Steps')
    plt.ylabel('Orientation')
    plt.legend()
    plt.tight_layout()
    plt.show()
