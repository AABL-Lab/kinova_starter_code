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

``autonomous.py``
- This file instructs the arm to go to a pre-specified joint position. As written, it will read the joint positions from a yaml file ``joint_states.yaml``. The yaml file contains a list of "goal positions" (i.e. joint positions). When creating an Autonomous instance, specify the name of the goal (in this case, ``goal1``, ``goal2``, or ``goal3``). 
- Note: You do not need to read joint positions from a yaml file. You could just as easily store them in a list. 
- Also note: this file specifies a robot pose based on the positions of all joints. Alternatively, you could just specify the position of the end effector.