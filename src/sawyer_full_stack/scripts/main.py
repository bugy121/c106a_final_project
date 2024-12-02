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
from moveit_commander import MoveGroupCommander

import os
import tf


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
    os.system("./go_to_joint_angles.py -q 0 -0.5 0 1.5 0 -1 1.7")
    os.system("./set_joint_speed.py")

def lookup_tag(tag_number, limb, kin, ik_solver, planner, args):
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
            trans = tfBuffer.lookup_transform("base", "ar_marker_"+str(tag_number), rospy.Time(0), rospy.Duration(10.0))
            found_ar_tag = True
        except Exception as e:
            print(e)
            print("Retrying ...")

            ### ATTEMPT 1 ###
            # # Try moving left to find the AR tag 
            # trans_base_arm = tfBuffer.lookup_transform('base', 'right_hand', rospy.Time(0), rospy.Duration(10.0))
            # curr_arm_pos = np.array([getattr(trans_base_arm.transform.translation, dim) for dim in ('x', 'y', 'z')])
            # new_arm_pos = np.array([curr_arm_pos[0], curr_arm_pos[1] + 0.1, curr_arm_pos[2]])
            # # debug:
            # # new_arm_pos = [0.6997413608390327, 0.04754310973086273, 0.21876114330591026]
            # print("CURRENT POS:", curr_arm_pos)
            # print("TARGET POS:", new_arm_pos)
            # # robot_trajectory = get_trajectory(limb, kin, ik_solver, arm_pos, args)
            # trajectory = LinearTrajectory(start_position=curr_arm_pos, goal_position=new_arm_pos, total_time=10)
            # path = MotionPath(limb, kin, ik_solver, trajectory)
            # robot_trajectory = path.to_robot_trajectory(args.num_way, True)

            # input('Move to starting position of trajectory')
            # breakpoint()
            # trajectory_start_pos = robot_trajectory.joint_trajectory.points[0].positions
            # plan = planner.plan_to_joint_pos(trajectory_start_pos)
            # planner.execute_plan(plan[1])       # index-1 is the joint_trajectory variable

            # input('Searching for AR tag')
            # planner.execute_plan(robot_trajectory)

            request = GetPositionIKRequest()
            request.ik_request.group_name = 'right_arm'
            link = 'right_gripper_tip'
            request.ik_request.ik_link_name = link
            request.ik_request.pose_stamped.header.frame_id = 'base'

            trans_base_arm = tfBuffer.lookup_transform('base', 'right_hand', rospy.Time(0), rospy.Duration(10.0))
            curr_arm_pos = np.array([getattr(trans_base_arm.transform.translation, dim) for dim in ('x', 'y', 'z')])
            curr_arm_orientation = np.array([getattr(trans_base_arm.transform.rotation, dim) for dim in ('x', 'y', 'z', 'w')])
            # new_arm_pos = np.array([curr_arm_pos[0], curr_arm_pos[1] + 0.1, curr_arm_pos[2]])    

            request.ik_request.pose_stamped.pose.position.x = curr_arm_pos[0] + 0.1
            request.ik_request.pose_stamped.pose.position.y = curr_arm_pos[1] - 0.2
            request.ik_request.pose_stamped.pose.position.z = curr_arm_pos[2]   
            request.ik_request.pose_stamped.pose.orientation.x = curr_arm_orientation[0]
            request.ik_request.pose_stamped.pose.orientation.y = curr_arm_orientation[1]
            request.ik_request.pose_stamped.pose.orientation.z = curr_arm_orientation[2]
            request.ik_request.pose_stamped.pose.orientation.w = curr_arm_orientation[3]

            compute_ik = rospy.ServiceProxy('compute_ik', GetPositionIK)

            try:
                response = compute_ik(request)
                print(response)
                group = MoveGroupCommander("right_arm")

                # Setting position and orientation target
                group.set_pose_target(request.ik_request.pose_stamped)

                # Plan IK
                plan = group.plan()
                input("Finding AR tag")
                group.execute(plan[1])
                
            except rospy.ServiceException as e:
                print("Service call failed: ", e)

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

def move_to(position, orientation=[0.0, 1.0, 0.0 ,0.0]):
    request = GetPositionIKRequest()
    request.ik_request.group_name = 'right_arm'
    link = 'right_gripper_tip'
    request.ik_request.ik_link_name = link
    request.ik_request.pose_stamped.header.frame_id = 'base'

    request.ik_request.pose_stamped.pose.position.x = position[0]
    request.ik_request.pose_stamped.pose.position.y = position[1]
    request.ik_request.pose_stamped.pose.position.z = position[2]
    request.ik_request.pose_stamped.pose.orientation.x = orientation[0]
    request.ik_request.pose_stamped.pose.orientation.y = orientation[1]
    request.ik_request.pose_stamped.pose.orientation.z = orientation[2]
    request.ik_request.pose_stamped.pose.orientation.w = orientation[3]

    compute_ik = rospy.ServiceProxy('compute_ik', GetPositionIK)

    try:
        response = compute_ik(request)
        print(response)
        group = MoveGroupCommander("right_arm")

        # Setting position and orientation target
        group.set_pose_target(request.ik_request.pose_stamped)

        # Plan IK
        plan = group.plan()
        print("Moving to", position, orientation)
        group.execute(plan[1])
        
    except rospy.ServiceException as e:
        print("Service call failed: ", e)


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
        'How many waypoints for the :obj:`moveit_msgs.msg.RobotTrajectory`.  Default: 300'
    )
    parser.add_argument('--log', action='store_true', help='plots controller performance')
    args = parser.parse_args()


    rospy.init_node('moveit_node')
    
    input('Tuck the arm')
    tuck()
    
    # this is used for sending commands (velocity, torque, etc) to the robot
    ik_solver = IK("base", "right_gripper_tip")
    limb = intera_interface.Limb("right")
    kin = sawyer_kinematics("right")


    if args.controller_name == "moveit":

        for i in range(5):
    
            # Moveit planner
            planner = PathPlanner('right_arm')

            # Lookup the AR tag position.
            # tag_pos = [lookup_tag(marker, limb, kin, ik_solver, planner, args) for marker in [i]]
            # [0.5542645905705363, -0.2634062656404528, -0.1872946122276515]
            tag_pos_and_orient = [lookup_tag(marker, limb, kin, ik_solver, planner, args) for marker in [i]]
            tag_pos = [data[0] for data in tag_pos_and_orient]
            tag_orient = [data[1] for data in tag_pos_and_orient]

            input(f"ITERATION {i} AR TAG POS: {tag_pos[0]}")
            # ITERATION 0 AR TAG POS: [ 0.78945311 -0.26583711 -0.23792368]
            # ITERATION 1 AR TAG POS: [ 0.7069352  -0.24560152 -0.18347042]
            
            # Get the trajectory from the robot arm to above the cube
            curr_tag_pos = [list(tag_pos[0])]
            curr_tag_pos[0][2] += 0.3
            curr_tag_pos[0][1] += 0.02
            target_pos = curr_tag_pos[0]
            input("Moving to above the block")
            move_to(target_pos)
            #robot_trajectory = get_trajectory(limb, kin, ik_solver, curr_tag_pos, args) 

            # input('Tuck the arm')
            # tuck()

            # input('Move to starting position of trajectory')
            # trajectory_start_pos = robot_trajectory.joint_trajectory.points[0].positions
            # plan = planner.plan_to_joint_pos(trajectory_start_pos)
            # planner.execute_plan(plan[1])       # index-1 is the joint_trajectory variable

            # input('Move to above the cube') 
            # planner.execute_plan(robot_trajectory)

            input('Open the gripper')
            gripper = intera_interface.Gripper('right_gripper')
            gripper.open()

            # Move down to the block, while also rotate gripper to match the block's orientation (screw motion)
            curr_tag_orient = list(tag_orient[0])
            (tag_row, tag_pitch, tag_yaw) = tf.transformations.euler_from_quaternion(curr_tag_orient)
            curr_arm_pos, curr_arm_orient = get_current_pos_orientation()
            (row, pitch, yaw) = tf.transformations.euler_from_quaternion(curr_arm_orient)
            target_tag_orientation = tf.transformations.quaternion_from_euler(row, pitch, yaw+tag_yaw%(np.pi/2))
            # (new_row, new_pitch, new_yaw) = tf.transformations.euler_from_quaternion(target_tag_orientation)
            input('Begin moving downward')
            curr_tag_pos = [list(tag_pos[0])]
            curr_tag_pos[0][2] += 0.05
            curr_tag_pos[0][1] += 0.02
            # robot_trajectory = get_trajectory(limb, kin, ik_solver, curr_tag_pos, args)
            # planner.execute_plan(robot_trajectory)
            move_to(curr_tag_pos[0], target_tag_orientation)

            input('Close the gripper')
            gripper = intera_interface.Gripper('right_gripper')        
            gripper.close()

            input('Begin moving upward')
            _, curr_arm_orient = get_current_pos_orientation()
            (row, pitch, yaw) = tf.transformations.euler_from_quaternion(curr_arm_orient)
            target_tag_orientation = tf.transformations.quaternion_from_euler(row, pitch, yaw-tag_yaw%(np.pi/2))
            # (new_row, new_pitch, new_yaw) = tf.transformations.euler_from_quaternion(target_tag_orientation)
            curr_tag_pos = [list(tag_pos[0])]
            curr_tag_pos[0][2] += 0.4
            # robot_trajectory = get_trajectory(limb, kin, ik_solver, curr_tag_pos, args)
            # planner.execute_plan(robot_trajectory)
            move_to(curr_tag_pos[0], target_tag_orientation)

            # curr_pos, curr_orient = get_current_pos_orientation()
            # print(curr_pos, curr_orient)
            # target_pos = np.array([curr_pos[0], -curr_pos[1], curr_pos[2]])
            if i == 0:
                target_pos = np.array([0.74, 0.3, 0.16])                # use hard-cose pos on first block
                target_orientation = np.array([0.0,1.0,0.0,0.0])
            else:
                target_pos = [prev_dropped_pos[0], prev_dropped_pos[1], 0.16]
                target_orientation = np.array([0.0,1.0,0.0,0.0])
            input('Move to stack postion')
            move_to(target_pos, target_orientation)

            curr_pos, curr_orient = get_current_pos_orientation()
            print(curr_pos, curr_orient)
            if i == 0:
                target_pos = np.array([curr_pos[0], curr_pos[1], curr_pos[2] - 0.45])
            else:
                target_pos = np.array([curr_pos[0], curr_pos[1], prev_dropped_pos[2] + 0.05])
            input('Moving down')
            move_to(target_pos, curr_orient)

            input('Open the gripper')
            gripper = intera_interface.Gripper('right_gripper')
            gripper.open()

            curr_pos, curr_orient = get_current_pos_orientation()
            print(curr_pos, curr_orient)
            target_pos = np.array([curr_pos[0], curr_pos[1], curr_pos[2] + 0.3])
            input('Moving up')
            move_to(target_pos, curr_orient)

            curr_pos, curr_orient = get_current_pos_orientation()
            print(curr_pos, curr_orient)
            target_pos = [curr_pos[0] + 0.2, curr_pos[1], curr_pos[2]]
            target_orientation = [0.0, np.sqrt(2)/2, 0.0, np.sqrt(2)/2]
            input('Rotate camera')
            move_to(target_pos, target_orientation)

            tag_pos = [lookup_tag(marker, limb, kin, ik_solver, planner, args) for marker in [i]]
            curr_tag_pos = list(tag_pos[0])
            prev_dropped_pos = curr_tag_pos
            print("Dropped postion:", prev_dropped_pos)

            input('Tuck the arm')
            tuck()
            # os.system("./go_to_joint_angles.py -q 0 -0.5 0 1.5 0 -1 1.7")
            # os.system("./set_joint_speed.py")
        
    else:
        controller = get_controller(args.controller_name, limb, kin)
        try:
            input('Press <Enter> to execute the trajectory using YOUR OWN controller')
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