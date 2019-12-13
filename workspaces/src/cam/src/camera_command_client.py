#!/usr/bin/env python 
# Aran Sena 2018 
# 
# Code example only, provided without guarantees 
# 
# Example for how to get both cameras streaming together 
# 
#### 
 
import rospy 
from intera_core_msgs.srv._IOComponentCommandSrv import IOComponentCommandSrv 
from intera_core_msgs.msg._IOComponentCommand import IOComponentCommand 
 
 
def camera_command_client(camera, status, timeout=0.0): 
    rospy.wait_for_service('/io/internal_camera/' + camera + '/command') 
    try: 
        cam_control = rospy.ServiceProxy('/io/internal_camera/' + camera + '/command', IOComponentCommandSrv) 
        cmd = IOComponentCommand() 
        cmd.time = rospy.Time.now() 
        cmd.op = 'set' 
        if status: 
            cmd.args = '{"signals": {"camera_streaming": {"data": [true], "format": {"type": "bool"}}}}' 
        else: 
            cmd.args = '{"signals": {"camera_streaming": {"data": [false], "format": {"type": "bool"}}}}' 
 
        resp = cam_control(cmd, timeout) 
        print resp 
 
    except rospy.ServiceException, e: 
        print "Service call failed: %s"%e 
 
 
if __name__ == '__main__': 
    rospy.init_node('camera_command_client') 
 
    camera_command_client(camera='head_camera', status=True) 
    camera_command_client(camera='right_hand_camera', status=True) 
 
