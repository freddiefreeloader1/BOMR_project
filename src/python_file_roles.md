### ROBOT
- path_following.py:
functions which help the calculations of drawing projections to a path and calculating what's up ahead

- robot.py:
has the Robot class, which stores the thymios position, path and angle
local avoidance function steer_avoid(),
async sensor functions, motor speed functions of the thymios


- robot_main.py:
init: start a node for the thymio/ lock the thymio
loop: update the motor speeds
close: close the node/ unlock the thymio


### CAMERA
- Astar.py
- computer_vision.py
- camera_main.py

### COMMON
- MAIN.py:


- common.py:
has the Quit exception, which is useful for exiting the while True loop from any place in the code