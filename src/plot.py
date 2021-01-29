#!/usr/bin/env python
import rospy
import numpy as np
import math
import cv2

from sensor_msgs.msg import LaserScan
import numpy as np
import matplotlib.pyplot as plt

# Fixing random state for reproducibility
np.random.seed(19680801)


class Laser2D:

  def callback(self, data):
    #print "hello"
    self.frame = np.zeros((500, 500,3), np.uint8)
    angle = data.angle_min
    x_list = []
    y_list = []
    for r in data.ranges:
      #change infinite values to 0
      if math.isinf(r) == True:
        r = 0
      x = math.trunc((r*30.0)*math.cos(angle+(-90.0*3.1416/180.0)))                         
      y = math.trunc((r * 30.0)*math.sin(angle + (-90.0*3.1416/180.0)))
      boundary = 500
      if y > boundary or y < -boundary or x<-boundary or x>boundary:
          x=0
          y=0
      cv2.line(self.frame,(250, 250),(x+250,y+250),(255,0,0),2)
      #x_list.append(angle)
      #y_list.append(r)
      x_list.append(x)
      y_list.append(y)
      angle= angle + data.angle_increment
      cv2.circle(self.frame, (250, 250), 2, (255, 255, 0))



    x_list = np.asarray(x_list)
    y_list = np.asarray(y_list)
    N = len(x_list)
    colors = np.random.rand(N)
    area = 5#(30 * np.random.rand(N))**2  # 0 to 15 point radii

    plt.scatter(x_list, y_list, s=area)#, c=colors, alpha=0.5)
    plt.show()
    #cv2.imshow('frame',self.frame)
    #cv2.waitKey(1)

  def __init__(self):
      self.frame = np.zeros((500, 500,3), np.uint8)
      rospy.init_node('laser_listener', anonymous=True)
      rospy.Subscriber("/laser_horizontal_front", LaserScan, self.callback)
      rospy.spin()

if __name__ == '__main__':
    x = Laser2D()
