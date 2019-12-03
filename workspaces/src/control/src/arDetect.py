import rospy
from ar_track_alvar_msgs.msg  import  AlvarMarkers
import numpy as np
import positionControl
from velocityControl import *
import coorConvert
import tf2_ros
#pointA,pointB = [x,y]
def generateLine(pointA,pointB):
    length = np.sqrt((pointA[0]-pointB[0]) ** 2 + (pointA[1]-pointB[1]) ** 2)
    numPoints = int(length / 0.01)
    dx = (- pointA[0] + pointB[0])/numPoints
    dy = (- pointA[1] + pointB[1])/numPoints
    points = [[pointA[0]+i*dx, pointA[1]+i*dy] for i in range(numPoints)]
    return points

#points = [[x1,y1],[x2,y2],...]
#In order to smooth the movment of the robot, add extra points.
def generateLines(points):
    pass

class coordinate(object):
    def __init__(self,ar_frame_1,ar_frame_2):
        self.tfBuffer1 = tf2_ros.Buffer()
        self.tfListener1 = tf2_ros.TransformListener(self.tfBuffer1)
        self.tfBuffer2 = tf2_ros.Buffer()
        self.tfListener2 = tf2_ros.TransformListener(self.tfBuffer2)
        self.dectAR()

    def dectAR(self):
        initialPoint = [0.49,0.58916,0.19804] #position for right_hand_camera
        goalPoint = [0.49,-0.62146,0.19804] #position for right_hand_camera
        [start_x,start_y,h] = initialPoint
        joints_position = coorConvert.cartesian2joint(start_x,start_y,h)
        positionControl.positionControl(jointCom=joints_position)

        controller = velocityControl(Limb("right"))
        try:            
            points = generateLine(initialPoint,goalPoint)
            plan = path(points,0.05,h).path_msg()          
            controller = velocityControl( Limb("right"))
            flag=controller.execute_path(plan)
            flag = 1
            if not flag:
                raise Exception("Execution failed")
        except Exception as e:
            print e



def test():
    rospy.init_node('testController', anonymous=True)
    coor = coordinate('base','base')

if __name__ == '__main__':
    test()


