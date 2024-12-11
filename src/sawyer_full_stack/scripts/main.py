#!/usr/bin/env python
"""
Starter script for 106a lab7. 
Author: Chris Correa
"""
import sys
import argparse
import numpy as np
import rospkg
import roslaunch

from paths.trajectories import LinearTrajectory, CircularTrajectory
from paths.paths import MotionPath
from paths.path_planner import PathPlanner
from controllers.controllers import ( 
    PIDJointVelocityController, 
    FeedforwardJointVelocityController
)
from utils.utils import *

from trac_ik_python.trac_ik import IK

import rospy
import tf2_ros
import intera_interface
from moveit_msgs.msg import DisplayTrajectory, RobotState
from sawyer_pykdl import sawyer_kinematics

from moveit_msgs.srv import GetPositionIK, GetPositionIKRequest, GetPositionIKResponse
from geometry_msgs.msg import PoseStamped
from moveit_commander import MoveGroupCommander, RobotCommander

import os
import tf
import intera_interface

from time import sleep

USING_AMIR = False

def tuck():
    """
    Tuck the robot arm to the start position. Use with caution
    """
    # if input('Would you like to tuck the arm? (y/n): ') == 'y':
    # rospack = rospkg.RosPack()
    # path = rospack.get_path('sawyer_full_stack')
    # launch_path = path + '/launch/custom_sawyer_tuck.launch'
    # uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
    # roslaunch.configure_logging(uuid)
    # launch = roslaunch.parent.ROSLaunchParent(uuid, [launch_path])
    # launch.start()
    # else:
    #     print('Canceled. Not tucking the arm.')
    # tuck_two_uh()
    # return
    os.system("./go_to_joint_angles.py -q 0 -0.5 0 1.5 0 -1 1.7")
    os.system("./set_joint_speed.py")

def lookup_tag(tag_number, limb, kin, ik_solver, planner, args, move_to):
    """
    Given an AR tag number, this returns the position of the AR tag in the robot's base frame.
    You can use either this function or try starting the scripts/tag_pub.py script.  More info
    about that script is in that file.  

    Parameters
    ----------
    tag_number : int

    Returns
    -------
    3x' :obj:`numpy.ndarray`
        tag position
    """

    tfBuffer = tf2_ros.Buffer()
    tfListener = tf2_ros.TransformListener(tfBuffer)

    found_ar_tag = False
    while not found_ar_tag:
    # for iter in interations and not found_ar_tag:
        try:
            # TODO: lookup the transform and save it in trans
            # The rospy.Time(0) is the latest available 
            # The rospy.Duration(10.0) is the amount of time to wait for the transform to be available before throwing an exception
            print("BEGIN TRY")
            trans = tfBuffer.lookup_transform("base", "ar_marker_"+str(tag_number), rospy.Time(0), rospy.Duration(0.5))
            found_ar_tag = True
            print("END TRY")
        except Exception as e:
            print(e)
            print("Retrying ...")

            cur_pos, cur_orient = get_current_pos_orientation()
            move_to(np.array([cur_pos[0]+0.1, cur_pos[1]-0.2, cur_pos[2]]), cur_orient, scale_factor=0.33)
            sleep(1)

    tag_pos = [getattr(trans.transform.translation, dim) for dim in ('x', 'y', 'z')]
    tag_orientation = [getattr(trans.transform.rotation, dim) for dim in ('x', 'y', 'z', 'w')]
    return (np.array(tag_pos), np.array(tag_orientation))

def get_trajectory(limb, kin, ik_solver, tag_pos, args):
    """
    Returns an appropriate robot trajectory for the specified task.  You should 
    be implementing the path functions in paths.py and call them here
    
    Parameters
    ----------
    task : string
        name of the task.  Options: line, circle, square
    tag_pos : 3x' :obj:`numpy.ndarray`
        
    Returns
    -------
    :obj:`moveit_msgs.msg.RobotTrajectory`
    """
    num_way = args.num_way
    task = args.task

    tfBuffer = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(tfBuffer)

    # lookup transform from base to right_hand
    try:
        trans = tfBuffer.lookup_transform('base', 'right_hand', rospy.Time(0), rospy.Duration(10.0))
    except Exception as e:
        print(e)

    # current x,y,z position of robot arm relative to base
    current_position = np.array([getattr(trans.transform.translation, dim) for dim in ('x', 'y', 'z')])
    current_orientation = np.array([getattr(trans.transform.rotation, dim) for dim in ('x', 'y', 'z', 'w')])

    if task == 'line':
        target_pos = tag_pos[0]
        # target_pos[2] += 0.4 #linear path moves to a Z position above AR Tag.
        print("CURRENT POSITION:", current_position)
        print("CURRENT ORITENTATION:", current_orientation)
        print("TARGET POSITION:", target_pos)
        trajectory = LinearTrajectory(start_position=current_position, goal_position=target_pos, total_time=3)
    elif task == 'circle':
        target_pos = tag_pos[0]
        target_pos[2] += 0.5
        print("TARGET POSITION:", target_pos)
        trajectory = CircularTrajectory(center_position=target_pos, radius=0.1, total_time=15)

    else:
        raise ValueError('task {} not recognized'.format(task))
    
    path = MotionPath(limb, kin, ik_solver, trajectory)
    robot_trajectory = path.to_robot_trajectory(num_way, True)
    return robot_trajectory

def get_controller(controller_name, limb, kin):
    """
    Gets the correct controller from controllers.py

    Parameters
    ----------
    controller_name : string
    tag_pos = [lookup_tag(marker) 
    Returns
    -------
    :obj:`Controller`
    """
    if controller_name == 'open_loop':
        controller = FeedforwardJointVelocityController(limb, kin)
    elif controller_name == 'pid':
        Kp = 0.2 * np.array([0.4, 2, 1.7, 1.5, 2, 2, 3])
        Kd = 0.01 * np.array([2, 1, 2, 0.5, 0.8, 0.8, 0.8])
        Ki = 0.01 * np.array([1.4, 1.4, 1.4, 1, 0.6, 0.6, 0.6])
        Kw = np.array([0.9, 0.9, 0.9, 0.9, 0.9, 0.9, 0.9])
        controller = PIDJointVelocityController(limb, kin, Kp, Ki, Kd, Kw)
    else:
        raise ValueError('Controller {} not recognized'.format(controller_name))
    return controller

def get_current_pos_orientation():
    tfBuffer = tf2_ros.Buffer()
    tfListener = tf2_ros.TransformListener(tfBuffer)
    trans_base_arm = tfBuffer.lookup_transform('base', 'right_hand', rospy.Time(0), rospy.Duration(10.0))
    curr_arm_pos = np.array([getattr(trans_base_arm.transform.translation, dim) for dim in ('x', 'y', 'z')])
    curr_arm_orientation = np.array([getattr(trans_base_arm.transform.rotation, dim) for dim in ('x', 'y', 'z', 'w')])
    return curr_arm_pos, curr_arm_orientation

def _move_to(target_position, target_orientation=[0.0, 1.0, 0.0 ,0.0], current_position=[0,0,0], scale_factor=0.33, planner=PathPlanner('right_arm'), group=MoveGroupCommander("right_arm")):
    # print("Moving to", position, orientation)
    request = GetPositionIKRequest()
    request.ik_request.group_name = 'right_arm'
    link = 'right_gripper_tip' if not USING_AMIR else 'stp_022312TP99620_tip_1'
    request.ik_request.ik_link_name = link
    request.ik_request.pose_stamped.header.frame_id = 'base'

    request.ik_request.pose_stamped.pose.position.x = target_position[0]
    request.ik_request.pose_stamped.pose.position.y = target_position[1]
    request.ik_request.pose_stamped.pose.position.z = target_position[2]
    request.ik_request.pose_stamped.pose.orientation.x = target_orientation[0]
    request.ik_request.pose_stamped.pose.orientation.y = target_orientation[1]
    request.ik_request.pose_stamped.pose.orientation.z = target_orientation[2]
    request.ik_request.pose_stamped.pose.orientation.w = target_orientation[3]

    compute_ik = rospy.ServiceProxy('compute_ik', GetPositionIK)

    try:
        response = compute_ik(request)

        # Setting position and orientation target
        start_pose = group.get_current_pose(link)
        group.set_pose_target(request.ik_request.pose_stamped)
        group.set_max_acceleration_scaling_factor(0.1)
        plan, _ = group.compute_cartesian_path([start_pose, request.ik_request.pose_stamped], 0.01, 0.01, avoid_collisions = True)
        # Plan IK
        # plan = group.plan()
        plan = group.retime_trajectory(
                        RobotCommander().get_current_state(), 
                        plan[1], 
                        scale_factor
                    )
        print("PLAN HERE:")
        print(plan)
        print("END PLAN")
        # plan = planner.retime_trajectory(plan, dt/9.0) # default trajectory takes 9 seconds, ig?
        group.execute(plan)
        
    except rospy.ServiceException as e:
        print("Service call failed: ", e)
    # return
    # ik_solver = IK("base", "right_gripper_tip" if not USING_AMIR else "stp_022312TP99620_tip_1")
    # limb = intera_interface.Limb("right")
    # kin = sawyer_kinematics("right")
    # curr_position, curr_orient = get_current_pos_orientation()
    # trajectory = LinearTrajectory(start_position=np.array(current_position), goal_position=np.array(target_position), total_time=9, orientation=np.array(curr_orient))
    # path = MotionPath(limb, kin, ik_solver, trajectory)
    # robo_traj = path.to_robot_trajectory(50, True)
    # planner = PathPlanner('right_arm')
    # planner.execute(robo_traj)

def main():
    """
    Examples of how to run me:
    python scripts/main.py --help <------This prints out all the help messages
    and describes what each parameter is
    python scripts/main.py -t line -ar_marker 3 -c torque --log
 
    You can also change the rate, timeout if you want
    """
    parser = argparse.ArgumentParser()
    parser.add_argument('-task', '-t', type=str, default='line', help=
        'Options: line, circle.  Default: line'
    )
    parser.add_argument('-ar_marker', '-ar', nargs='+', help=
        'Which AR marker to use.  Default: 1'
    )
    parser.add_argument('-controller_name', '-c', type=str, default='moveit', 
        help='Options: moveit, open_loop, pid.  Default: moveit'
    )
    parser.add_argument('-rate', type=int, default=200, help="""
        This specifies how many ms between loops.  It is important to use a rate
        and not a regular while loop because you want the loop to refresh at a
        constant rate, otherwise you would have to tune your PD parameters if 
        the loop runs slower / faster.  Default: 200"""
    )
    parser.add_argument('-timeout', type=int, default=None, help=
        """after how many seconds should the controller terminate if it hasn\'t already.  
        Default: None"""
    )
    parser.add_argument('-num_way', type=int, default=50, help=
        'How many waypoints for the :obj:`moveit_msgs.msg.RobotTrajectory`.  Default: 50'
    )
    parser.add_argument('--log', action='store_true', help='plots controller performance')
    parser.add_argument('--amir', action='store_true', help='sets tf frames for use with Amir robot')
    parser.add_argument('--table_height', '-T', type=float, default=-0.1, help='sets the height of the table')
    parser.add_argument('--wait', action='store_true', help='waits for [enter] with `input()` before each move')

    args = parser.parse_args()

    input_fixed = lambda prompt: input(prompt) if args.wait else None

    global USING_AMIR
    USING_AMIR = args.amir

    rospy.init_node('moveit_node')
    
    input_fixed('Tuck the arm')
    tuck()
    # this is used for sending commands (velocity, torque, etc) to the robot
    ik_solver = IK("base", "right_gripper_tip" if not USING_AMIR else 'stp_022312TP99620_tip_1')
    limb = intera_interface.Limb("right")
    kin = sawyer_kinematics("right")
    planner = PathPlanner('right_arm')
    movegroup = MoveGroupCommander('right_arm')
    move_to = lambda *args, **kwargs: _move_to(*args, **kwargs, planner=planner, group=movegroup)
    if args.controller_name == "moveit":

        for i, tag_number in enumerate(args.ar_marker):
    
            # Moveit planner
            planner = PathPlanner('right_arm')

            # Lookup the AR tag position.
            # tag_pos = [lookup_tag(marker, limb, kin, ik_solver, planner, args) for marker in [i]]
            # [0.5542645905705363, -0.2634062656404528, -0.1872946122276515]
            
            tag_pos, tag_orient = lookup_tag(tag_number, limb, kin, ik_solver, planner, args, move_to = move_to)
            # tag_pos = [data[0] for data in tag_pos_and_orient]
            # tag_orient = [data[1] for data in tag_pos_and_orient]

            print(f"ITERATION {i} AR TAG POS: {tag_pos}")
            # ITERATION 0 AR TAG POS: [ 0.78945311 -0.26583711 -0.23792368]
            # ITERATION 1 AR TAG POS: [ 0.7069352  -0.24560152 -0.18347042]
            
            # Get the trajectory from the robot arm to above the cube
            curr_tag_pos = [list(tag_pos)]
            curr_tag_pos[0][2] += 0.3
            curr_tag_pos[0][1] += 0.02
            target_pos = curr_tag_pos[0]
            input_fixed("Moving to above the block")
            move_to(target_pos)
            #robot_trajectory = get_trajectory(limb, kin, ik_solver, curr_tag_pos, args) 

            # input_fixed('Tuck the arm')
            # tuck()

            # input_fixed('Move to starting position of trajectory')
            # trajectory_start_pos = robot_trajectory.joint_trajectory.points[0].positions
            # plan = planner.plan_to_joint_pos(trajectory_start_pos)
            # planner.execute_plan(plan[1])       # index-1 is the joint_trajectory variable

            # input_fixed('Move to above the cube') 
            # planner.execute_plan(robot_trajectory)

            input_fixed('Open the gripper')
            gripper = intera_interface.Gripper('right_gripper')
            gripper.open()

            # Move down to the block, while also rotate gripper to match the block's orientation (screw motion)
            curr_tag_orient = list(tag_orient)
            (tag_row, tag_pitch, tag_yaw) = tf.transformations.euler_from_quaternion(curr_tag_orient)
            curr_arm_pos, curr_arm_orient = get_current_pos_orientation()
            (row, pitch, yaw) = tf.transformations.euler_from_quaternion(curr_arm_orient)
            target_tag_orientation = tf.transformations.quaternion_from_euler(row, pitch, yaw+tag_yaw%(np.pi/2))
            # (new_row, new_pitch, new_yaw) = tf.transformations.euler_from_quaternion(target_tag_orientation)
            input_fixed('Begin moving downward')
            curr_tag_pos = [list(tag_pos)]
            # curr_tag_pos[0][2] += 0.05
            curr_tag_pos[0][2] = args.table_height  # HARDCODE TABLE HEIGHT
            curr_tag_pos[0][1] += 0.02
            # robot_trajectory = get_trajectory(limb, kin, ik_solver, curr_tag_pos, args)
            # planner.execute_plan(robot_trajectory)
            move_to(curr_tag_pos[0], target_tag_orientation)
            sleep(0.5)

            input_fixed('Close the gripper')
            gripper = intera_interface.Gripper('right_gripper')        
            gripper.close()

            input_fixed('Begin moving upward')
            _, curr_arm_orient = get_current_pos_orientation()
            (row, pitch, yaw) = tf.transformations.euler_from_quaternion(curr_arm_orient)
            target_tag_orientation = tf.transformations.quaternion_from_euler(row, pitch, yaw-tag_yaw%(np.pi/2))
            # (new_row, new_pitch, new_yaw) = tf.transformations.euler_from_quaternion(target_tag_orientation)
            curr_tag_pos = [list(tag_pos)]
            # curr_tag_pos[0][2] += 0.4
            z_clearance = 0.2
            curr_tag_pos[0][2] = args.table_height + 0.0508*i + z_clearance
            # robot_trajectory = get_trajectory(limb, kin, ik_solver, curr_tag_pos, args)
            # planner.execute_plan(robot_trajectory)
            move_to(curr_tag_pos[0], target_tag_orientation)

            # if i == 0:
            #     target_pos = np.array([0.74, 0.3, 0.16])                # use hard-cose pos on first block
            #     target_orientation = np.array([0.0,1.0,0.0,0.0])
            # else:
            #     target_pos = [prev_dropped_pos[0], prev_dropped_pos[1], 0.16]   # only care about x,y position, height is above the stack location
            #     target_orientation = np.array([0.0,1.0,0.0,0.0])

            target_pos = np.array([0.74, 0.3, args.table_height + 0.0508*i + z_clearance])
            target_orientation = np.array([0.0, 1.0, 0.0, 0.0])
            input_fixed('Move to above the stack postion')
            move_to(target_pos, target_orientation, scale_factor=0.5)

            # curr_pos, curr_orient = get_current_pos_orientation()

            target_pos = np.array([target_pos[0], target_pos[1], args.table_height + 0.0508*i])
            input_fixed('Moving down')
            move_to(target_pos, target_orientation, scale_factor=0.11)
            sleep(0.5)


            input_fixed('Open the gripper')
            gripper = intera_interface.Gripper('right_gripper')
            gripper.open(0.018)

            curr_pos, curr_orient = get_current_pos_orientation()
            prev_dropped_pos = curr_pos
            print(curr_pos, curr_orient)
            target_pos = np.array([curr_pos[0], curr_pos[1], curr_pos[2] + 0.08])
            input_fixed('Moving up')
            move_to(target_pos, curr_orient)

            # cur_pos, cur_orient = get_current_pos_orientation()
            target_pos = np.array([target_pos[0], target_pos[1]-0.3, target_pos[2]])
            input_fixed("moving away from stack")
            move_to(target_pos, curr_orient, scale_factor=0.5)

            # curr_pos, curr_orient = get_current_pos_orientation()
            # print(curr_pos, curr_orient)
            # target_pos = [curr_pos[0] + 0.2, curr_pos[1], curr_pos[2]]
            # target_orientation = [0.0, np.sqrt(2)/2, 0.0, np.sqrt(2)/2]
            # input_fixed('Rotate camera')
            # # move_to(target_pos, target_orientation)
            # limb = intera_interface.Limb('right')
            # target_joint_angles = {
            #     'right_j0': 0.367197265625, 
            #     'right_j1': -0.683083984375, 
            #     'right_j2': -0.320525390625, 
            #     'right_j3': 1.2404873046875, 
            #     'right_j4': 0.3167783203125, 
            #     'right_j5': -0.5252392578125, 
            #     'right_j6': 1.72457421875
            # }
            # curr_joint_angles = limb.joint_angles()     # dict: joint_name --> joint_angle
            # while not all(abs(curr_joint_angles[name] - target_joint_angles[name]) < 0.1 for name in curr_joint_angles):
            #     limb.set_joint_positions(target_joint_angles)
            #     curr_joint_angles = limb.joint_angles()

            # Extract dropped postion
            # tag_pos_and_orient = [lookup_tag(marker, limb, kin, ik_solver, planner, args) for marker in [tag_number]]
            # tag_pos = [data[0] for data in tag_pos_and_orient]
            # tag_orient = [data[1] for data in tag_pos_and_orient]
            # curr_tag_pos = list(tag_pos[0])
            # prev_dropped_pos = curr_tag_pos
            # print("Dropped postion:", prev_dropped_pos)

            input_fixed('Tuck the arm')
            tuck()
        
    else:
        controller = get_controller(args.controller_name, limb, kin)
        try:
            input_fixed('Press <Enter> to execute the trajectory using YOUR OWN controller')
        except KeyboardInterrupt:
            sys.exit()
        # execute the path using your own controller.
        done = controller.execute_path(
            robot_trajectory, 
            rate=args.rate, 
            timeout=args.timeout, 
            log=args.log
        )
        if not done:
            print('Failed to move to position')
            sys.exit(0)


if __name__ == "__main__":
    main()


# Publish trajectory to the move_group/display_planned_path topic in Rviz
# pub = rospy.Publisher('move_group/display_planned_path', DisplayTrajectory, queue_size=10)
# disp_traj = DisplayTrajectory()
# disp_traj.trajectory.append(robot_trajectory)
# disp_traj.trajectory_start = RobotState()
# pub.publish(disp_traj)

# if args.controller_name != "moveit":
#     plan = planner.retime_trajectory(plan, 0.3)



# each block-stacking motion consists of
# 1. look for current block
# 2. move gripper above the block
# 3. move down
# 4. close gripper
# 5. move up
# 6. move sideways to somewhere above the stack
# 7. move down (depend on current height of stack)
# 8. release gripper
# 9. go back to home position

# while (not stacked)
#   look at current state
#   plan = ... (stack block i)
#   execute = ...