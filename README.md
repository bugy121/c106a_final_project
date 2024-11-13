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

Commands to get the sawyer to move toward block with ar tag:
(Run after ssh-ed into sawyer, each command in a new terminal)
```
source ~ee106a/sawyer_setup.bash
rosrun intera_interface joint_trajectory_action_server.py

source ~ee106a/sawyer_setup.bash
roslaunch sawyer_moveit_config sawyer_moveit.launch electric_gripper:=true

source ~ee106a/sawyer_setup.bash
rosrun intera_examples camera_display.py -c right_hand_camera

source ~ee106a/sawyer_setup.bash
source devel/setup.bash
roslaunch sawyer_full_stack sawyer_camera_track.launch

source ~ee106a/sawyer_setup.bash
source devel/setup.bash
cd src/sawyer_full_stack/scripts
rosrun intera_interface enable_robot.py -e
python main.py -task line -ar_marker 1 --log
```
