

### ROBOT CONSTANTS ###
#pathing
PATH_LOOKAHEAD = 0.08

#driving units (-500,500)
ROBOT_FOLLOW_FORWARD_SPEED = 250
ROBOT_FOLLOW_STEER_AMOUNT = 150

#avoiding constants
ROBOT_AVOID_FORWARD_SPEED = 200
ROBOT_AVOID_SENSOR_GAIN = 12    #how much speed do i gain for avoiding an obstacle
ROBOT_AVOID_SENSOR_RESCIND = 4  #amount the sensor steers away the opposite wheel
ROBOT_AVOID_SENSOR_STOP = 15    #how much does the central sensor reverse the robot
