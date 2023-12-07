import math

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

PROX_DANGER_MIN = 10
PROX_DANGER_MAX = 20

STATE_COOLDOWN = 4

#sensor constants
MOTOR_SENSOR_TO_SPINS = 2*math.pi/(4.6*400)
MOTOR_SENSOR_TO_MPS = 0.0003175
ACCELERATION_SENSOR_TO_MPSS = 9.81/21.0

PROXIMITIY_SMOOTHING = 0.7     #use PROX_SMOOTHING of a sensor, and (1-P_SM) of the neighboring prox. sensors ( 1 - use only [0],[2],[4])
### KALMAN ###
#noise components 
ACCEL_NOISE = 10*9.81/23
VEL_NOISE = 0.001
ROT_NOISE = 0.005
POS_NOISE = 0.005
SPIN_NOISE = 0.1