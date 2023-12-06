from computer_vision import *
from robot_main import init_robot_position, get_time
from Astar_coord import *
from Astar import * 
from camera_main import CameraClose, CameraInit, CameraLoop
from robot_main import RobotClose, RobotInit, RobotLoop
from common import Quit, SharedData,shared

import matplotlib.pyplot as plt
import numpy as np

# This is a simple function that hard sets the robots data.
# TODO: initialize the kalman after this step, perhaps create a 'RobotStart(pos)'

def convert_camera_to_robot(position = None, angle = None, path = None):
    if angle is not None: angle = -angle 
    if position is not None: 
        position = (position[0]/1000.0, -position[1]/1000.)
    if path is not None:
        path = np.array([(p[0]/1000.0, -p[1]/1000.) for p in path])
    
    return position, angle, path


def main():
    global shared
    CameraInit()
    RobotInit()
    kalman_history = []
    absolute_history = []
    kalman_history_orientation = []
    absolute_orientation = []
    # replace loop with return
    try:
        while True:
            # Run the camera loop
            CameraLoop(shared)
            # If the camera has data for the robot, update it.
            if((shared.robot is None and len(shared.metric_path) > 0)):
                a,b,c = convert_camera_to_robot(shared.thymio_position,shared.thymio_angle,shared.metric_path)
                init_robot_position(shared,a,b,c) #CAMERA ANGLE IS CLOCKWISE!!!

            
            if(shared.thymio_position is not None and shared.robot is not None):
                new_pos, new_angle, _ = convert_camera_to_robot(shared.thymio_position, shared.thymio_angle)
                absolute_history.append(new_pos)
                absolute_orientation.append(new_angle)
                
                shared.robot.kalman.update_position(new_pos, get_time())
                shared.robot.kalman.update_heading(new_angle, get_time())
                
                kalman_history.append(shared.robot.kalman.get_position())
                kalman_history_orientation.append(shared.robot.kalman.get_rotation())
                shared.thymio_position = None

            # If the robot was given a path, start running.
            if(len(shared.metric_path) > 0 and not(shared.robot is None)):
                RobotLoop(shared)
                #print((shared.robot.odometry.x,shared.robot.odometry.y),shared.end,shared.robot.odometry.angle )

    except Quit:
        pass

    # Plotting
    absolute_history = np.array(absolute_history)
    kalman_history = np.array(kalman_history)
    metric_path = np.array(c)
    absolute_orientation = np.array(absolute_orientation)
    kalman_history_orientation = np.array(kalman_history_orientation)
    path_display = np.array(shared.path_shared)

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
    # A quit has been called (unlock that thymio!!!!)
    CameraClose()
    RobotClose()

if __name__ == "__main__":
    main()