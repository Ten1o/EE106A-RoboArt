#!/usr/bin/python
import numpy as np
from positionControl import *
import rospy
from arDetect import *

class draw(object):
    #inputFig = np.array()
    #size = 255 or 1000
    def __init__(self, coord, inputFig, size, h_offset=0.25):
        self.coord = coord
        self.originalFig = inputFig
        self.scalar = min(abs(coord.coord1.transform.translation.x-coord.coord2.transform.translation.x),\
            abs(coord.coord1.transform.translation.y-coord.coord2.transform.translation.y))
        self.h = -min(coord.coord1.transform.translation.z,coord.coord2.transform.translation.z)
        self.h_offset = h_offset
        self.size = size
        self.fig = inputFig
        for i in range(len(inputFig)):
            for j in range(len(inputFig[i])):
                self.fig[i][j][0] = inputFig[i][j][0]/size*self.scalar + coord.origin[0]
                self.fig[i][j][1] = inputFig[i][j][1]/size*self.scalar + coord.origin[1]
                # self.fig[i][j][0] = inputFig[i][j][0]
                # self.fig[i][j][1] = inputFig[i][j][1]

        print(self.fig)
        # self.draw()

    def liftArm(self,currentPos):
         #position for right_hand_camera
        [start_x,start_y,start_h] = currentPos
        joints_position = coorConvert.cartesian2joint(start_x,start_y,start_h-0.05,mode=1)
        positionControl.positionControl(jointCom=joints_position)

    def transferNextPos(self,nextPos):
        r=rospy.Rate(0.5)
        [start_x,start_y,start_h] = nextPos
        joints_position = coorConvert.cartesian2joint(start_x,start_y,start_h-0.05,mode=1)
        positionControl.positionControl(jointCom=joints_position)
        r.sleep()
        [start_x,start_y,start_h] = nextPos
        joints_position = coorConvert.cartesian2joint(start_x,start_y,start_h,mode=1)
        positionControl.positionControl(jointCom=joints_position)
        r.sleep()

    def draw(self):
        controller = velocityControl( Limb("right"))
        r=rospy.Rate(1)
        print()
        for i in range(len(self.fig)):
            nextPos = [self.fig[i][0][0], self.fig[i][0][1], self.h + self.h_offset] 
            self.transferNextPos(nextPos)

            # while not rospy.is_shutdown():
            for j in range(len(self.fig[i])-1):
                try:
                    pointA = [self.fig[i][j][0], self.fig[i][j][1]]
                    pointB = [self.fig[i][j+1][0], self.fig[i][j+1][1]]
                    # print('___________________________________')
                    # print(pointA)
                    # print('___________________________________')
                    # print(pointB)   
                    # print('___________________________________')    
                    points = arDetect.generateLine(pointA,pointB)
                    my_path = path(points,0.023,self.h+self.h_offset)
                    plan = my_path.path_msg()       
                    if my_path.valid:
                        flag=controller.execute_path(plan)
                        pass
                    else:
                        flag=0

                    if not flag:
                        raise Exception("Execution failed")
                except Exception as e:
                    print e

            
            currentPos = [self.fig[i][-1][0], self.fig[i][-1][1], self.h+self.h_offset]
            self.liftArm(currentPos)
 


            # [start_x,start_y,h] = [0.72, 0.01, -0.00288962+0.35]
            # joints_position = coorConvert.cartesian2joint(start_x,start_y,h)
            # positionControl.positionControl(jointCom=joints_position)


def test():
    rospy.init_node('testController', anonymous=True)
    coor = coordinate('ar_marker_4','ar_marker_5')
    #inputFig = [[[39.0,144.0],[51.0,146.0],[49.0,170.0],[52.0,183.0],[81.0,220.0],[77.0,219.0],[51.0,181.0],[51.0,146.0],[38.0,144.0],[34.0,164.0],[39.0,144.0],[35.0,136.0],[29.0,143.0],[28.0,152.0],[30.0,170.0],[38.0,189.0],[30.0,170.0],[30.0,141.0],[26.0,140.0],[23.0,153.0],[27.0,180.0],[36.0,194.0],[49.0,202.0],[49.0,213.0],[61.0,218.0],[47.0,211.0],[43.0,216.0],[52.0,228.0],[70.0,237.0],[69.0,241.0],[65.0,240.0],[84.0,247.0],[90.0,256.0],[87.0,249.0],[71.0,243.0],[69.0,237.0],[52.0,228.0],[43.0,216.0],[45.0,210.0],[32.0,197.0],[39.0,207.0],[47.0,212.0],[50.0,209.0],[50.0,203.0],[37.0,195.0],[28.0,183.0],[21.0,151.0],[24.0,152.0],[34.0,123.0],[27.0,139.0],[31.0,141.0],[39.0,128.0],[35.0,108.0],[31.0,108.0],[13.0,75.0],[31.0,108.0],[35.0,108.0],[37.0,86.0],[15.0,70.0],[8.0,73.0],[20.0,98.0],[18.0,103.0],[30.0,118.0],[22.0,139.0],[14.0,142.0],[14.0,154.0],[0.0,147.0],[5.0,152.0],[15.0,153.0],[9.0,130.0],[13.0,112.0],[9.0,128.0],[13.0,142.0],[17.0,141.0],[17.0,135.0],[17.0,141.0],[23.0,137.0],[23.0,131.0],[15.0,132.0],[13.0,123.0],[15.0,132.0],[26.0,128.0],[20.0,112.0],[26.0,125.0],[30.0,118.0],[28.0,112.0],[12.0,99.0],[12.0,96.0],[16.0,102.0],[21.0,99.0],[13.0,89.0],[8.0,73.0],[10.0,69.0],[2.0,54.0],[10.0,70.0],[17.0,71.0],[37.0,86.0],[41.0,74.0],[47.0,74.0],[62.0,48.0],[75.0,38.0],[85.0,36.0],[85.0,39.0],[95.0,40.0],[115.0,51.0],[112.0,59.0],[99.0,56.0],[80.0,65.0],[73.0,75.0],[76.0,85.0],[68.0,90.0],[69.0,94.0],[79.0,91.0],[69.0,94.0],[68.0,90.0],[88.0,77.0],[117.0,65.0],[124.0,66.0],[125.0,70.0],[105.0,81.0],[107.0,91.0],[115.0,95.0],[115.0,99.0],[121.0,99.0],[123.0,92.0],[128.0,93.0],[129.0,97.0],[128.0,93.0],[119.0,90.0],[123.0,91.0],[122.0,98.0],[114.0,99.0],[112.0,103.0],[123.0,104.0],[109.0,103.0],[98.0,107.0],[97.0,103.0],[102.0,100.0],[98.0,103.0],[100.0,108.0],[112.0,103.0],[115.0,94.0],[119.0,93.0],[109.0,93.0],[105.0,81.0],[99.0,82.0],[95.0,88.0],[88.0,88.0],[95.0,88.0],[111.0,75.0],[104.0,75.0],[80.0,90.0],[104.0,75.0],[112.0,76.0],[125.0,70.0],[124.0,66.0],[103.0,70.0],[78.0,84.0],[73.0,75.0],[87.0,60.0],[99.0,56.0],[112.0,58.0],[118.0,51.0],[142.0,66.0],[181.0,103.0],[191.0,123.0],[191.0,140.0],[196.0,143.0],[190.0,150.0],[174.0,158.0],[154.0,159.0],[134.0,144.0],[124.0,147.0],[123.0,173.0],[99.0,190.0],[82.0,193.0],[71.0,185.0],[76.0,183.0],[89.0,188.0],[110.0,175.0],[106.0,172.0],[104.0,176.0],[92.0,179.0],[75.0,177.0],[72.0,172.0],[78.0,167.0],[79.0,160.0],[83.0,164.0],[87.0,161.0],[82.0,167.0],[89.0,166.0],[99.0,156.0],[105.0,159.0],[109.0,153.0],[111.0,160.0],[97.0,171.0],[81.0,172.0],[97.0,171.0],[112.0,158.0],[110.0,151.0],[105.0,159.0],[101.0,156.0],[106.0,154.0],[97.0,154.0],[99.0,156.0],[85.0,166.0],[81.0,159.0],[87.0,155.0],[73.0,170.0],[75.0,177.0],[92.0,179.0],[106.0,172.0],[110.0,174.0],[120.0,166.0],[95.0,186.0],[83.0,188.0],[76.0,182.0],[73.0,189.0],[82.0,193.0],[94.0,192.0],[123.0,173.0],[126.0,158.0],[121.0,144.0],[123.0,147.0],[133.0,144.0],[127.0,124.0],[133.0,144.0],[154.0,159.0],[183.0,155.0],[195.0,144.0],[200.0,147.0],[206.0,186.0],[199.0,207.0],[205.0,196.0],[205.0,163.0],[210.0,163.0],[212.0,185.0],[209.0,156.0],[196.0,123.0],[196.0,112.0],[169.0,75.0],[169.0,70.0],[181.0,66.0],[191.0,79.0],[202.0,84.0],[210.0,100.0],[209.0,104.0],[202.0,84.0],[191.0,79.0],[151.0,36.0],[111.0,14.0],[102.0,16.0],[127.0,31.0],[147.0,50.0],[127.0,31.0],[103.0,19.0],[104.0,12.0],[93.0,1.0],[104.0,13.0],[151.0,36.0],[180.0,65.0],[178.0,69.0],[168.0,70.0],[155.0,54.0],[196.0,112.0],[196.0,123.0],[209.0,162.0],[204.0,162.0],[200.0,147.0],[191.0,140.0],[182.0,149.0],[152.0,153.0],[165.0,151.0],[165.0,148.0],[172.0,151.0],[173.0,147.0],[169.0,147.0],[183.0,137.0],[184.0,132.0],[179.0,141.0],[175.0,135.0],[158.0,141.0],[144.0,139.0],[144.0,134.0],[163.0,136.0],[167.0,131.0],[163.0,136.0],[155.0,136.0],[155.0,130.0],[165.0,127.0],[169.0,130.0],[172.0,125.0],[169.0,130.0],[166.0,127.0],[155.0,130.0],[155.0,136.0],[144.0,134.0],[143.0,137.0],[158.0,141.0],[175.0,135.0],[185.0,127.0],[185.0,122.0],[170.0,119.0],[167.0,121.0],[170.0,125.0],[167.0,122.0],[160.0,125.0],[170.0,119.0],[183.0,120.0],[186.0,125.0],[175.0,134.0],[178.0,142.0],[173.0,150.0],[182.0,149.0],[190.0,142.0],[192.0,137.0],[188.0,128.0],[191.0,123.0],[186.0,111.0],[142.0,66.0],[106.0,46.0],[111.0,42.0],[153.0,68.0],[108.0,40.0],[105.0,46.0],[85.0,39.0],[87.0,33.0],[93.0,35.0],[71.0,26.0],[56.0,12.0],[42.0,15.0],[33.0,11.0],[33.0,1.0],[33.0,11.0],[53.0,14.0],[46.0,1.0],[64.0,21.0],[87.0,32.0],[65.0,45.0],[47.0,74.0],[41.0,74.0],[37.0,40.0],[43.0,23.0],[46.0,23.0],[19.0,19.0],[11.0,23.0],[10.0,30.0],[0.0,23.0],[8.0,29.0],[17.0,29.0],[20.0,35.0],[16.0,48.0],[11.0,46.0],[5.0,50.0],[0.0,49.0],[19.0,44.0],[18.0,40.0],[14.0,43.0],[10.0,36.0],[2.0,44.0],[2.0,40.0],[5.0,43.0],[7.0,39.0],[5.0,33.0],[10.0,34.0],[13.0,42.0],[20.0,38.0],[20.0,33.0],[11.0,28.0],[6.0,14.0],[0.0,13.0],[0.0,3.0],[6.0,18.0],[15.0,23.0]]]
    #inputFig = [[[0.54, 0.30],[0.48, 0.08],[0.72, 0.01]]]
    # inputFig =  [[[373.0, 205.0], [665.0, 172.0], [668.0, 686.0]]]
    # inputFig = [[[358.0, 194.0], [365.0, 710.0]], [[373.0, 205.0], [665.0, 172.0], [668.0, 686.0]]]
    inputFig = [[[358.0, 194.0], [365.0, 710.0]], [[373.0, 205.0], [665.0, 172.0], [668.0, 686.0]], [[383.0, 362.0], [553.0, 344.0]], [[385.0, 519.0], [583.0, 495.0]], [[110.0, 732.0], [915.0, 704.0]]]
    drawIns = draw(coor, inputFig, 1000.0, h_offset=0.143)

    print('ARframe1 is ',coor.coord1)
    print('ARframe2 is ',coor.coord2)
    print('origin is ', coor.origin)
if __name__ == '__main__':
    test()
