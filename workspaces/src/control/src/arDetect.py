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

def generateArc():
    pass

#points = [[x1,y1],[x2,y2],...]
#In order to smooth the movment of the robot, add extra points.
def generateLines(points):
    pass

class coordinate(object):
    def __init__(self,ar_frame1,ar_frame2):
        self.ar_frame1 = ar_frame1
        self.ar_frame2 = ar_frame2
        self.tfBuffer1 = tf2_ros.Buffer()
        self.tfListener1 = tf2_ros.TransformListener(self.tfBuffer1)
        self.tfBuffer2 = tf2_ros.Buffer()
        self.tfListener2 = tf2_ros.TransformListener(self.tfBuffer2)
        self.coord1 = []
        self.coord2 = []
        self.origin = 0
        self.dectAR()
        self.setOrigin()



    def dectAR(self):
        initialPoint = [0.49,0.58916,0.19804] #position for right_hand_camera
        goalPoint = [0.49,-0.62146,0.19804] #position for right_hand_camera
        [start_x,start_y,h] = initialPoint
        joints_position = coorConvert.cartesian2joint(start_x,start_y,h,mode=0)
        positionControl.positionControl(jointCom=joints_position)

        arInfo = {'names':[self.ar_frame1,self.ar_frame2],'coords':[self.coord1,self.coord2]}


        try:            
            points = generateLine(initialPoint,goalPoint)
            plan = path(points,0.05,h,mode=0).path_msg()          
            controller = velocityControl(Limb("right"),arDetectInfo=arInfo)
            flag=controller.execute_path(plan)
            self.coord1 = controller.arDetectInfo['coords'][0]
            self.coord2 = controller.arDetectInfo['coords'][1]
            flag = 1
            if not flag:
                raise Exception("Execution failed")
        except Exception as e:
            print e

    def buildFrame(self):
        while not rospy.is_shutdown():
            try:
                self.coord1 = self.tfBuffer1.lookup_transform('base', self.ar_frame1, rospy.Time())
                self.coord2 = self.tfBuffer2.lookup_transform('base', self.ar_frame2, rospy.Time())
                break
            except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
                pass

    def setOrigin(self):
        if(self.coord1!=[] and self.coord2 !=[]):
            if(self.coord1.transform.translation.x < self.coord2.transform.translation.x):
                self.origin = 1
            elif (self.coord1.transform.translation.x > self.coord2.transform.translation.x):
                self.origin = 2
            else:
                self.origin = -1
                rospy.loginfo("Origin Set Fault.")
        else:
            print("AR Coords not found")

    def coordTransformation(self):
        pass



def test():
    rospy.init_node('testController', anonymous=True)
    coor = coordinate('ar_marker_3','ar_marker_4')
    print('ARframe1 is ',coor.coord1)
    print('ARframe2 is ',coor.coord2)

if __name__ == '__main__':
    test()


