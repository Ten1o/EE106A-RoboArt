#!/usr/bin/env python
"""
Controller Class for Lab 8
Author: Valmik Prabhu, Chris Correa
"""

import rospy
import sys
import numpy as np
import itertools

import baxter_interface
import intera_interface
from intera_interface import Limb

from moveit_msgs.msg import RobotTrajectory

from moveit_msgs.msg import OrientationConstraint
from geometry_msgs.msg import PoseStamped

from path_planner import PathPlanner


class velocityControl(object):
    """
    A controller object

    Fields:
    _Kp: 7x' ndarray of proportional constants
    _Ki: 7x' ndarray of integral constants
    _Kd: 7x' ndarray of derivative constants
    _Kw: 7x' ndarray of antiwindup constants
    _LastError: 7x' ndarray of previous position errors
    _LastTime: Time from start at which LastError was updated (in sec)
    _IntError: 7x' ndarray of integrated error values
    _path: a moveit_msgs/RobotTrajectory message
    _curIndex: the current index in the path
    _maxIndex: maximum index in the path
    _limb: baxter_interface.Limb or intera_interface.Limb

    _times: For Plotting
    _actual_positions: For Plotting
    _actual_velocities: For Plotting
    _target_positions: For Plotting
    _target_velocities: For Plotting

    Methods:
    __init__(self, Kp, Ki, Kd, Kw): constructor

    """

    def __init__(self, Kp, Ki, Kd, Kw, limb):
        """
        Constructor:

        Inputs:
        Kp: 7x' ndarray of proportional constants
        Ki: 7x' ndarray of integral constants
        Kd: 7x' ndarray of derivative constants
        Kw: 7x' ndarray of antiwindup constants
        limb: baxter_interface.Limb or sawyer_interface.Limb
        """

        # If the node is shutdown, call this function
        rospy.on_shutdown(self.shutdown)

        self._Kp = Kp
        self._Ki = Ki
        self._Kd = Kd
        self._Kw = Kw

        self._LastError = np.zeros(len(Kd))
        self._LastTime = 0;
        self._IntError = np.zeros(len(Ki))
        self._ring_buff = [0.0, 0.0, 0.0]

        self._path = RobotTrajectory()
        self._curIndex = 0;
        self._maxIndex = 0;

        self._limb = limb

        # For Plotting:
        self._times = list()
        self._actual_positions = list()
        self._actual_velocities = list()
        self._target_positions = list()
        self._target_velocities = list()

    
    def shutdown(self):
        """
        Code to run on shutdown. This is good practice for safety
        """
        rospy.loginfo("Stopping Controller")

        # Set velocities to zero
        self._limb.set_joint_velocities(dict(itertools.izip(self._limb.joint_names(), np.zeros(len(self._limb.joint_names())))))
        rospy.sleep(0.1)

    def execute_path(self, path, timeout=100.0, log=True):
        """
        Execute a given path

        Inputs:
        path: a moveit_msgs/RobotTrajectory message
        timeout: max time the controller will run
        log: should the controller display a plot
        
        """

        self._path = path

        self._curIndex = 0
        self._maxIndex = len(self._path.joint_trajectory.points)-1

        startTime = rospy.Time.now()

        # Set the last error as zero for t = 0
        self._LastError = np.zeros(len(self._Kd))
        self._LastTime = 0.0

        # Set the integral of positions to zero
        self._IntError = np.zeros(len(self._Ki))

        # Set your ring buffer to zero
        self._ring_buff = [0.0, 0.0, 0.0]

        # Reset plot values
        self._times = list()
        self._actual_positions = list()
        self._actual_velocities = list()
        self._target_positions = list()
        self._target_velocities = list()
        
        r = rospy.Rate(200)

        while not rospy.is_shutdown():
            # Find the time from start
            t = (rospy.Time.now() - startTime).to_sec()

            # If the controller has timed out, stop moving and return false
            if timeout is not None and t >= timeout:
                # Set velocities to zero
                self._limb.set_joint_velocities(dict(itertools.izip(self._limb.joint_names(), np.zeros(len(self._limb.joint_names())))))
                return False

            # Get the input for this time
            u = self.step_control(t)

            # Set the joint velocities
            self._limb.set_joint_velocities(dict(itertools.izip(self._limb.joint_names(), u)))
            # Sleep for a defined time (to let the robot move)
            r.sleep()

            # Once the end of the path has been reached, stop moving and break
            if self._curIndex >= self._maxIndex:
                # Set velocities to zero
                self._limb.set_joint_velocities(dict(itertools.izip(self._limb.joint_names(), np.zeros(len(self._limb.joint_names())))))
                break

        if log:
            import matplotlib.pyplot as plt

            times = np.array(self._times)
            actual_positions = np.array(self._actual_positions)
            actual_velocities = np.array(self._actual_velocities)
            target_positions = np.array(self._target_positions)
            target_velocities = np.array(self._target_velocities)
            plt.figure()
            joint_num = len(self._path.joint_trajectory.joint_names)
            for joint in range(joint_num):
                plt.subplot(joint_num,2,2*joint+1)
                plt.plot(times, actual_positions[:,joint], label='Actual')
                plt.plot(times, target_positions[:,joint], label='Desired')
                plt.xlabel("Time (t)")
                if(joint is 0):
                    plt.ylabel(self._path.joint_trajectory.joint_names[joint] + " Position Error")
                else:
                    plt.ylabel(self._path.joint_trajectory.joint_names[joint])
                plt.legend()

                plt.subplot(joint_num,2,2*joint+2)
                plt.plot(times, actual_velocities[:,joint], label='Actual')
                plt.plot(times, target_velocities[:,joint], label='Desired')
                plt.xlabel("Time (t)")
                if(joint is 0):
                    plt.ylabel(self._path.joint_trajectory.joint_names[joint] + " Velocity Error")
                else:
                    plt.ylabel(self._path.joint_trajectory.joint_names[joint])
                plt.legend()

            print "Close the plot window to continue"
            plt.show()

        return True

    def step_control(self, t):
        """
        Return the control input given the current controller state at time t

        Inputs:
        t: time from start in seconds

        Output:
        u: 7x' ndarray of velocity commands
        
        """
        # Make sure you're using the latest time
        while (not rospy.is_shutdown() and self._curIndex < self._maxIndex and self._path.joint_trajectory.points[self._curIndex+1].time_from_start.to_sec() < t+0.001):
            self._curIndex = self._curIndex+1


        current_position = np.array([self._limb.joint_angles()[joint_name] for joint_name in self._path.joint_trajectory.joint_names])
        current_velocity = np.array([self._limb.joint_velocities()[joint_name] for joint_name in self._path.joint_trajectory.joint_names])

        if self._curIndex < self._maxIndex:
            time_low = self._path.joint_trajectory.points[self._curIndex].time_from_start.to_sec()
            time_high = self._path.joint_trajectory.points[self._curIndex+1].time_from_start.to_sec()

            target_position_low = np.array(self._path.joint_trajectory.points[self._curIndex].positions)
            target_velocity_low = np.array(self._path.joint_trajectory.points[self._curIndex].velocities)

            target_position_high = np.array(self._path.joint_trajectory.points[self._curIndex+1].positions)
            target_velocity_high = np.array(self._path.joint_trajectory.points[self._curIndex+1].velocities)

            target_position = target_position_low + (t - time_low)/(time_high - time_low)*(target_position_high - target_position_low)
            target_velocity = target_velocity_low + (t - time_low)/(time_high - time_low)*(target_velocity_high - target_velocity_low)

        else:
            target_position = np.array(self._path.joint_trajectory.points[self._curIndex].positions)
            target_velocity = np.array(self._path.joint_trajectory.points[self._curIndex].velocities)

        # For Plotting
        self._times.append(t)
        self._actual_positions.append(current_position)
        self._actual_velocities.append(current_velocity)
        self._target_positions.append(target_position)
        self._target_velocities.append(target_velocity)


        # Feed Forward Term
        u_ff = target_velocity

        # Error Term
        error = target_position - current_position

        # Integral Term
        self._IntError = self._Kw * self._IntError + error
        
        # Derivative Term
        dt = t - self._LastTime
        # We implement a moving average filter to smooth the derivative
        self._ring_buff[2] = self._ring_buff[1]
        self._ring_buff[1] = self._ring_buff[0]
        self._ring_buff[0] = (error - self._LastError) / dt
        ed = np.mean(self._ring_buff)

        # Save terms for the next run
        self._LastError = error
        self._LastTime = t


        ###################### YOUR CODE HERE #########################

        # Note, you should load the Kp, Ki, Kd, and Kw constants with
        # self._Kp
        # and so on. This is better practice than hard-coding

        u = u_ff+self._Kp*error+self._Kd*ed+self._Ki*self._IntError

        ###################### YOUR CODE END ##########################

        return u


def main():
    planner = PathPlanner("right_arm")
    Kp = 0.2 * np.array([0.4, 2, 1.7, 1.5, 2, 2, 3])
    Kd = 0.01 * np.array([2, 1, 2, 0.5, 0.8, 0.8, 0.8])
    Ki = 0.01 * np.array([1.4, 1.4, 1.4, 1, 0.6, 0.6, 0.6])
    Kw = np.array([0.9, 0.9, 0.9, 0.9, 0.9, 0.9, 0.9])
    controller = velocityControl(Kp, Ki, Kd, Kw, Limb("right"))


    while not rospy.is_shutdown():
        try:
            goal_1 = PoseStamped()
            goal_1.header.frame_id = "base"

            #x, y, and z position
            goal_1.pose.position.x = 0.8
            goal_1.pose.position.y = 0.05
            goal_1.pose.position.z = -0.23

            #Orientation as a quaternion
            goal_1.pose.orientation.x = 0.0
            goal_1.pose.orientation.y = -1.0
            goal_1.pose.orientation.z = 0.0
            goal_1.pose.orientation.w = 0.0

            # Might have to edit this . . . 
            plan = planner.plan_to_pose(goal_1, list())
            print(plan)
            
            raw_input("Press <Enter> to move the right arm to goal pose 1: ")
            controller = velocityControl(Kp, Ki, Kd, Kw, Limb("right"))
            flag=controller.execute_path(plan)
            if not flag:
                raise Exception("Execution failed")
        except Exception as e:
            print e
        else:
            break


if __name__ == '__main__': 
    rospy.init_node('velocityControl_node')
    main()



    





