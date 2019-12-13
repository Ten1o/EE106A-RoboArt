#!/usr/bin/env python

# Copyright (c) 2013-2018, Rethink Robotics Inc.
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
SDK Joint Position Example: keyboard
"""
import argparse
import rospy

import intera_interface
import intera_external_devices

#from baxter_interface import gripper as robot_gripper
from intera_interface import gripper as robot_gripper

from intera_interface import CHECK_VERSION
import numpy as np

#jointCom = {'joint_num':'joint_angle', ...}
#gripperCom=["close" or "open" or "calibrate"]
def positionControl(jointCom={}, gripperCom=""):

    #Limb Configuration
    rp = intera_interface.RobotParams()
    valid_limbs = rp.get_limb_names()
    if not valid_limbs:
        rp.log_message(("Cannot detect any limb parameters on this robot. "
                        "Exiting."), "ERROR")
        return
    arg_fmt = argparse.RawDescriptionHelpFormatter
    parser = argparse.ArgumentParser(formatter_class=arg_fmt,
                                     description=main.__doc__)
    parser.add_argument(
        "-l", "--limb", dest="limb", default=valid_limbs[0],
        choices=valid_limbs,
        help="Limb on which to run the joint position keyboard example"
    )
    args = parser.parse_args(rospy.myargv()[1:])

    limb = intera_interface.Limb(args.limb)
    # print(intera_interface)

    #Gripper Detection
    try:
        gripper = intera_interface.Gripper(args.limb + '_gripper')
    except:
        has_gripper = False
        rospy.loginfo("The electric gripper is not detected on the robot.")
    else:
        has_gripper = True

    joints = limb.joint_names()


    #Joint Set
    #This function will set the joint angle for only one joint.
    def set_j(limb, joint_name, angle):
        current_position = limb.joint_angle(joint_name)
        joint_command = {joint_name: angle}

        while (abs(current_position - angle) > 0.01 ):
            current_position = limb.joint_angle(joint_name)
            limb.set_joint_positions(joint_command)

    def set_all_j(limb,jointCom):
        
        names = ['right_j0', 'right_j1', 'right_j2', 'right_j3', 'right_j4', 'right_j5', 'right_j6']
        dic ={}
        for i in range (len(names)):
            dic[names[i]] = jointCom[i]
        current_position = limb.joint_angles()
        curr_angles = np.array([current_position[key] for key in sorted(current_position.keys())])
        dest_angles = np.array([jointCom[key] for key in sorted(jointCom.keys())])
        error = np.sum(np.abs(curr_angles-dest_angles))
        r = rospy.Rate(200)
        while (error > 0.02 ):
            current_position = limb.joint_angles()
            curr_angles = np.array([current_position[key] for key in sorted(current_position.keys())])
            error = np.sum(np.abs(curr_angles-dest_angles))
            limb.set_joint_positions(dic)
            r.sleep()

    #Trajectory Set
    #Set the trajectory of all joints at the same time.
    #joint_names = [str]
    #angles, velocities, accelerations = [float]
    def set_t(limb, joint_names, angles, velocities, accelerations):
        current_position = limb.joint_angles()  #This func return a dictionary of {joint:angle} pairs.    
        # velocities = [.1,.1,.1,.1,.1,.1,.1]
        # accelerations = [.1,.1,.1,.1,.1,.1,.1]
        
        limb.set_joint_trajectory(joint_names, angles, velocities, accelerations)


    #Gripper Set
    def set_g(action):
        if has_gripper:
            if action == "close":
                gripper.close()
                print('Gripper closing...')
            elif action == "open":
                gripper.open()
                print('Gripper opening...')
            elif action == "calibrate":
                gripper.calibrate()
                print('Gripper calibrating...')
    
    #Execution
    # for key, val in jointCom.items():

    #     set_j(limb, joints[key], val)
    if(jointCom != {}):
        set_all_j(limb,jointCom)
        set_g(gripperCom)
    else:
        set_g(gripperCom)




    #Keyboard manipulation

    # bindings = {
    #     '1': (set_j, [limb, joints[0], 0.1], joints[0]+" increase"),
    #     'q': (set_j, [limb, joints[0], -0.1], joints[0]+" decrease"),
    #     '2': (set_j, [limb, joints[1], 0.1], joints[1]+" increase"),
    #     'w': (set_j, [limb, joints[1], -0.1], joints[1]+" decrease"),
    #     '3': (set_j, [limb, joints[2], 0.1], joints[2]+" increase"),
    #     'e': (set_j, [limb, joints[2], -0.1], joints[2]+" decrease"),
    #     '4': (set_j, [limb, joints[3], 0.1], joints[3]+" increase"),
    #     'r': (set_j, [limb, joints[3], -0.1], joints[3]+" decrease"),
    #     '5': (set_j, [limb, joints[4], 0.1], joints[4]+" increase"),
    #     't': (set_j, [limb, joints[4], -0.1], joints[4]+" decrease"),
    #     '6': (set_j, [limb, joints[5], 0.1], joints[5]+" increase"),
    #     'y': (set_j, [limb, joints[5], -0.1], joints[5]+" decrease"),
    #     '7': (set_j, [limb, joints[6], 0.1], joints[6]+" increase"),
    #     'u': (set_j, [limb, joints[6], -0.1], joints[6]+" decrease")
    #  }
    # if has_gripper:
    #     bindings.update({
    #     '8': (set_g, "close", side+" gripper close"),
    #     'i': (set_g, "open", side+" gripper open"),
    #     '9': (set_g, "calibrate", side+" gripper calibrate")
    #     })

    # print("Controlling joints. Press ? for help, Esc to quit.")
    return


def main():
    """RSDK Joint Position Example: Keyboard Control

    Use your dev machine's keyboard to control joint positions.

    Each key corresponds to increasing or decreasing the angle
    of a joint on Sawyer's arm. The increasing and descreasing
    are represented by number key and letter key next to the number.
    """
#     epilog = """
# See help inside the example with the '?' key for key bindings.
#     """




    # rp = intera_interface.RobotParams()
    # valid_limbs = rp.get_limb_names()
    # if not valid_limbs:
    #     rp.log_message(("Cannot detect any limb parameters on this robot. "
    #                     "Exiting."), "ERROR")
    #     return
    # arg_fmt = argparse.RawDescriptionHelpFormatter
    # parser = argparse.ArgumentParser(formatter_class=arg_fmt,
    #                                  description=main.__doc__,
    #                                  epilog=epilog)
    # parser.add_argument(
    #     "-l", "--limb", dest="limb", default=valid_limbs[0],
    #     choices=valid_limbs,
    #     help="Limb on which to run the joint position keyboard example"
    # )
    # args = parser.parse_args(rospy.myargv()[1:])









    print("Initializing node... ")
    rospy.init_node("positionControl")
    print("Getting robot state... ")
    rs = intera_interface.RobotEnable(CHECK_VERSION)
    init_state = rs.state().enabled

    def clean_shutdown():
        print("\nExiting example.")

    rospy.on_shutdown(clean_shutdown)

    rospy.loginfo("Enabling robot...")
    rs.enable()

    # jointCom={0:-0.02516796875, 1:1.19958203125, 2:-3.0417177734375, \
    # 3:1.609376953125, 4:0.032142578125, 5:-1.742896484375, 6:-1.635921875}
    #5:.433978515625
    # jointCom={0:0.27098046875, 1:-0.503345703125, 2:0, 3:1.4947724609375, 4:0, 5:0.6492998046875, 6:0}
    jointCom={0:0, 1:0, 2:0, 3:0, 4:0, 5:0, 6:0}

    # positionControl({},"calibrate")  
    # positionControl({},"open")
    positionControl({},"open")
    #positionControl(jointCom) 


    print("Done.")


if __name__ == '__main__':
    
    main()
