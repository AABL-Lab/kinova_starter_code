# kinova_starter_code

``direct_control.py``
- This file lets you directly control the Gen2 arm via an xbox controller. 
- Note that you can't teleoperate the arm until it has been homed (pass ``homed=True`` to your DirectControl instance). 
- Before running this code, you will need to get the robot started up. To do that, use the command ``roslaunch kinova_starter_code launch_arm.launch``

### config
This is where we store the mappings between controllers and the commands that are sent to the robot. ``XYZMode.yaml`` contains the mappings between buttons on an xbox controller and the x/y/z velocity commands sent to the robot.