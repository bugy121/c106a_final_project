# c106a_final_project
start roscore
```
roscore
```

ssh into the sawyer
```
source ~ee106a/sawyer_setup.bash
```

enable the robot
```
rosrun intera_interface enable_robot.py -e
```

Start everything
```
source ~ee106a/sawyer_setup.bash

rosrun intera_interface joint_trajectory_action_server.py &
roslaunch sawyer_moveit_config sawyer_moveit.launch electric_gripper:=true &
rosrun intera_examples camera_display.py -c right_hand_camera &
catkin make; # if you haven't done this already
source devel/setup.bash;
roslaunch sawyer_full_stack sawyer_camera_track.launch &
```

to run the code:
```
source ~ee106a/sawyer_setup.bash
source devel/setup.bash
cd src/sawyer_full_stack/scripts
rosrun intera_interface enable_robot.py -e
python main.py -ar_marker 1 2 3 4 5 --table_height 0.14 
```
If using Amir, pass the `--amir` flag. 
To wait for \[enter\] before each movement, pass the `--wait` flag.
