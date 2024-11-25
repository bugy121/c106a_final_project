#! /usr/bin/env python
# Copyright (c) 2013-2018, Rethink Robotics Inc.
# Copyright (c) 2024 Regents of the University of California
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""
Resets the joint speed of a Sawyer robot after a tuck.
"""
import time
import argparse

import rospy

import intera_interface
import intera_external_devices

from intera_interface import CHECK_VERSION


def fix_speed(side):
    limb = intera_interface.Limb(side)

    joints = limb.joint_names()

    def set_j(limb, joint_name, delta):
        current_position = limb.joint_angle(joint_name)
        joint_command = {joint_name: current_position + delta}
        # Set the joint speed to the default with every command
        limb.set_joint_position_speed(0.3)
        limb.set_joint_positions(joint_command)

    print("Fixing joint speed.")
    # Give the robot time to get ready
    time.sleep(1)
    # Ask for a movement with zero position change -- this will reset the speed
    set_j(limb, joints[0], 0)
    # Sleep again to ensure the movement completes before we shut down
    time.sleep(1)
    rospy.signal_shutdown("Complete!")

def main():
    """Resets the joint speed of the robot after a tuck.
    """
    rp = intera_interface.RobotParams()
    valid_limbs = rp.get_limb_names()
    if not valid_limbs:
        rp.log_message(("Cannot detect any limb parameters on this robot. "
                        "Exiting."), "ERROR")
        return

    print("Initializing node... ")
    rospy.init_node("sdk_set_joint_speed")
    print("Getting robot state... ")
    rs = intera_interface.RobotEnable(CHECK_VERSION)
    init_state = rs.state().enabled

    def clean_shutdown():
        print("\nComplete!")

    rospy.on_shutdown(clean_shutdown)

    rospy.loginfo("Enabling robot...")
    rs.enable()
    fix_speed(valid_limbs[0])
    print("Done.")


if __name__ == '__main__':
    main()
