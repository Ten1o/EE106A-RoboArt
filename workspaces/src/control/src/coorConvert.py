#!/usr/bin/python
import numpy as np
from positionControl import *
import rospy
L1=0.4
L2=0.4
#mode = 1: gripper orthogonal to ground
#mode = 0: gripper parallel to ground
def polar2joint(r,h=0.43,angle=0.1817734375, mode=1):
    # r:float , h(height):float 
    # r is the horizontal distance between joint_1 and joint_5 
    # h is the height between joint_1 and joint_5
    if ( np.sum((-(4*(h**2) + 25*np.square(r))*(4*h**2 + 25*np.square(r) - 16))< 0) >0 ):
        rospy.loginfo("Position Error(sqrt < 0), stop running.")
        return 'fault'
    a=2*np.arctan((8*h + (-(4*(h**2) + 25*(r**2))*(4*h**2 + 25*r**2 - 16))**(0.5))/(4*h**2 + 25*r**2 + 20*r) - (16*h)/(4*h**2 + 25*r**2 + 20*r))
    b=2*np.arctan((8*h + (-(4*h**2 + 25*r**2)*(4*h**2 + 25*r**2 - 16))**(0.5))/(4*h**2 + 25*r**2 + 20*r))
    t1 = -a
    t3 = a+b
    t5 = -b+np.pi/2*mode

    # return theta1, theta3, theta5, which can be applied directly in positionControl()
    return {0:angle, 1:t1, 2:0, 3:t3, 4:0, 5:t5, 6:0}

def cartesian2polar(x,y,h=0.43):
    r_square = np.add(np.square(x),np.square(y))
    r_offset = 0.0815457 # joint0_x -joint1_x
    s = 0.105122 + 0.0504428  # distance between joint5 and joint6 + joint 0_y and joint 1_y
    s_square = s**2
    if np.sum((r_square-s_square)<0) > 0:
        exit()
    robot_r = np.sqrt(r_square - s_square)  
    theta = np.arctan2(y,x)
    theta_offset =np.arcsin(s/np.sqrt(r_square))
    #return [np.sqrt(r_square),theta,h]
    return [robot_r-r_offset,theta-theta_offset,h]

def cartesian2joint(x,y,h=0.43,mode=1):
    [r,theta,h] = cartesian2polar(x,y,h)
    ret = polar2joint(r,h,theta,mode)

    return ret
    
def test():    
    print("Initializing node... ")
    rospy.init_node("Trajectory")
    rs = intera_interface.RobotEnable(CHECK_VERSION)
    init_state = rs.state().enabled
    def clean_shutdown():
        print("\nExiting Trajectory.")
    rospy.on_shutdown(clean_shutdown)

    rospy.loginfo("Enabling robot...")
    rs.enable()

    jointCom=cartesian2joint(0,0,0.43)
    positionControl(jointCom, "open")  
    print("Done.")

if __name__ == '__main__': 
    test()