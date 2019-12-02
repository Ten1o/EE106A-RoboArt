#!/usr/bin/env python
import rospy
import sys
import numpy as np
import itertools
import intera_interface
from intera_interface import Limb

from moveit_msgs.msg import RobotTrajectory
from moveit_msgs.msg import JointTrajectoryPoint
from geometry_msgs.msg import PoseStamped


 
class path(object):
	"""docstring for pathPlanner"""
	#points [[x1,y1],[x2,y2]]
	#velocity: float
	def __init__(self, points,velocity,h):

		self.points = self.xy2rt(np.array(points)) #array([[r1,theta1],[r2,theta2],...])
		self.velocity = velocity
		self.height = h
		self.positions = self.len2angle()
		self.number = len(points)
		self.time_interval = self.time_interval(np.array(points))
		self.velocities = velocities(self) # n*7
	def len2angle(self):
	    # r:float , h(height):float 
	    # r is the horizontal distance between joint_1 and joint_5 
	    # h is the height between joint_1 and joint_5
	    r=self.points[:,0]
	    theta=self.points[:,1]
	    h=self.height
	    a=2*np.arctan((8*h + np.sqrt(-(4*(h**2) + 25*np.square(r))*(4*h**2 + 25*np.square(r) - 16)))/(4*h**2 + 25*np.square(r) + 20*r) - (16*h)/(4*h**2 + 25*np.square(r) + 20*r))
	    b=2*np.arctan((8*h + np.sqrt(-(4*h**2 + 25*np.square(r))*(4*h**2 + 25*np.square(r) - 16)))/(4*h**2 + 25*np.square(r) + 20*r))
	    t1 = -a
	    t3 = a+b
	    t5 = np.pi/2-b
	    # return theta1, theta3, theta5, which can be applied directly in positionControl()
	    ret=np.vstack((theta, t1, np.zeros(self.number), t3, np.zeros(self.number), t5, np.zeros(self.number)))
	    return = np.transpose(ret)

	def xy2rt(self,points):
		x=points[:,0]
		y=points[:,1]
		r=np.sqrt(np.add(np.square(x),np.square(y)))
		t=np.arctan2(y,x)
		ret=np.transpose(np.vstack((r,t)))
		return ret

	def time_interval(self,points):
		#points: array([[x1,y1],[x2,y2],...])
		a = np.copy(points)
		a = np.delete(a,0,axis=0)
		b = np.copy(points)
		b = np.delete(b,-1,axis=0)
		dis = a-b
		x = dis[:,0]
		y = dis[:,1]
		r=np.sqrt(np.add(np.square(x),np.square(y)))
		time_interval = r/self.velocity
		ret=np.hstack((0,time_interval))
		return ret
	def velocities(self):
		time=np.delete(self.time_interval,0)
		a = np.copy(self.positions)
		b = np.copy(self.positions)
		a = np.delete(a,0,axis=0)
		b = np.delete(b,-1,axis=0)
		dis = a-b
		velocities = dis/time.reshape(self.number-1,1)
		velocities =np.vstack((velocities,np.zeros(7)))
		return velocities


	def path_msg(self,joint_names):
		time_from_start = np.cumsum(self.time_interval)
		path = RobotTrajectory()
		path.joint_trajectory.joint_names = joint_names
		path.joint_trajectory.points = [JointTrajectoryPoint()]*self.number
		for i in range(self.number):
			point=JointTrajectoryPoint()
			point.positions = self.positions[i]
			point.velocities = self.velocities[i]
			point.duration = time_from_start[i]
			path.joint_trajectory.points[i]=point
		return path



		
