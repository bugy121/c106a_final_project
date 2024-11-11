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
rosrun intera_interface joint_trajectory_action_server.py
roslaunch sawyer_moveit_config sawyer_moveit.launch electric_gripper:=true
rosrun intera_examples camera_display.py -c right_hand_camera
roslaunch sawyer_full_stack sawyer_camera_track.launch

cd src/sawyer_full_stack/scripts
python main.py -task {line/circle} -ar_marker {number} --log
```
