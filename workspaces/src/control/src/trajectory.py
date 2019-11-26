#!/usr/bin/python
import numpy as np
from positionControl import *
import rospy
L1=0.4
L2=0.4
def len2angle(r,h=0.43,angle=0.1817734375):
    # r:float , h(height):float 
    # r is the horizontal distance between joint_1 and joint_5 
    # h is the height between joint_1 and joint_5
    a=2*np.arctan((8*h + (-(4*(h**2) + 25*(r**2))*(4*h**2 + 25*r**2 - 16))**(0.5))/(4*h**2 + 25*r**2 + 20*r) - (16*h)/(4*h**2 + 25*r**2 + 20*r))
    b=2*np.arctan((8*h + (-(4*h**2 + 25*r**2)*(4*h**2 + 25*r**2 - 16))**(0.5))/(4*h**2 + 25*r**2 + 20*r))
    t1 = -a
    t3 = a+b
    t5 = np.pi/2-b

    # return theta1, theta3, theta5, which can be applied directly in positionControl()
    return {0:angle, 1:t1, 2:0, 3:t3, 4:0, 5:t5, 6:0}


print("Initializing node... ")
rospy.init_node("Trajectory")
rs = intera_interface.RobotEnable(CHECK_VERSION)
init_state = rs.state().enabled
def clean_shutdown():
    print("\nExiting Trajectory.")
rospy.on_shutdown(clean_shutdown)

rospy.loginfo("Enabling robot...")
rs.enable()

jointCom=len2angle(0.4)
positionControl(jointCom, "open")  
jointCom=len2angle(0.5)
positionControl(jointCom, "open")
jointCom=len2angle(0.6)
positionControl(jointCom, "open")    


print("Done.")