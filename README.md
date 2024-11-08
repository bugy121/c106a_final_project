# c106a_final_project

Commands to get the sawyer to move toward ar code:
  1. rosrun intera_interface joint_trajectory_action_server.py
  2. roslaunch sawyer_moveit_config sawyer_moveit.launch electric_gripper:=true
  3. rosrun intera_examples camera_display.py -c right_hand_camera
  4. roslaunch sawyer_full_stack sawyer_camera_track.launch
  5. python main.py -task {line/circle} -ar_marker {number} --log
