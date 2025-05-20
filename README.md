# kinova_starter_code



### config
This is where we store the mappings between controllers and the commands that are sent to the robot. ``XYZMode.yaml`` contains the mappings between buttons on an xbox controller and the x/y/z velocity commands sent to the robot.

### scripts
This is where we keep the collision scenes for the robots. It's where we define where obstacles are within the environment. When you run the collision scene node and give a trajectory for the robot to follow, it will know to avoid these obstacles. For example, to start the collision scene you would type ``rosrun kinova_starter_code setup_tablebot_collision_scene.py``, 

### launch
Launch files will automatically rosrun any nodes that are specified. This can make it easier to keep track of the nodes you need to run. 

### src
``direct_control.py``
- This file lets you directly control the Gen2 arm via an xbox controller. 
- Note that you can't teleoperate the arm until it has been homed (pass ``homed=True`` to your DirectControl instance). 
- Before running this code, you will need to get the robot started up. To do that, use the command ``roslaunch kinova_starter_code launch_arm.launch``