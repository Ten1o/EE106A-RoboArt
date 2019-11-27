#!/usr/bin/env python
import rospy
import sys
import numpy as np
import itertools
import intera_interface
from intera_interface import Limb

from moveit_msgs.msg import RobotTrajectory

from geometry_msgs.msg import PoseStamped

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
    return [angle, t1, 0, t3, 0, t5, 0]

class path(object):
	"""docstring for pathPlanner"""
	#points [{'x':float,'y':float}]
	#velocity: float
	def __init__(self, points,velocity,h):
		self.points = points
		self.velocity = velocity
		self.height = h
		for 