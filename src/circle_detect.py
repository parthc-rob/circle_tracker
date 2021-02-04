#!/usr/bin/env python
import rospy
import numpy as np
import math
import cv2

from sensor_msgs.msg import LaserScan

from sensor_msgs.msg import PointCloud
from geometry_msgs.msg import Point32
from std_msgs.msg import Header
import numpy as np
import matplotlib.pyplot as plt
from visualization_msgs.msg import Marker

import circle_fit
from kf_lib import *

from scipy.stats import norm

# Fixing random state for reproducibility
np.random.seed(19680801)

class PointScan:
  def __init__(self):
    self.x = None
    self.y = None
    self.r = None
    self.theta = None
    self.point32 =  Point32()

class Scan:
  def __init__(self):
    self.points = []
    self.angle_min = None
    self.angle_max = None
    self.range_min = None
    self.range_max = None
    self.angle_span = None
    self.minsegsize = 4 # min num of points to consider segment
    self.scan_ratio = 10.0 

  def initialize(self, x_list, y_list, r_list, angle_list):
    self.points = []
    for x, y, r, theta in zip(x_list, y_list, r_list, angle_list):
      pt = PointScan()
      pt.x = x
      pt.y = y
      pt.r = r
      pt.theta = theta 
      self.points.append(pt)

class Circle:
  def __init__(self):
    self.radius = 0.28
    self.a = None
    self.b = None
  def initialize( self, inputs ):
    self.a = inputs[0] 
    self.b = inputs[1]
    self.radius = inputs[2]

def preprocess_scan(data): # scan from polar coordinates -> x,y
  
  angle = data.angle_min
  x_list = []
  y_list = []

  angle_list = [data.angle_min + i*data.angle_increment for i in range( len(data.ranges)) ] 

  cos_angle_list = [math.cos(a) for a in angle_list]
  sin_angle_list = [math.sin(a) for a in angle_list]
  x_list = np.multiply(np.array(data.ranges), cos_angle_list)
  y_list = np.multiply(np.array(data.ranges), sin_angle_list)

  full_scan = Scan()
  full_scan.initialize(x_list, y_list, data.ranges, angle_list)
  full_scan.angle_min = data.angle_min
  full_scan.angle_max = data.angle_max
  full_scan.angle_span = data.angle_max - data.angle_min
  return full_scan 

def get_segments(scan): # list of Point's - x, y, r, theta
  #print("getting segments")
  iter = 0
  segment_iter = 0

  segment_list = []
  segment = []
  
  #print("scanpoints")
  #print(len(scan.points))
  scan_str = ""
  while (iter < len(scan.points)):
  
    if iter == segment_iter:
      segment.append(scan.points[iter])
      iter += 1
    else: # detect breaks
      angle_diff =  scan.points[iter].theta - scan.points[iter-1].theta
      # max adaptive breakpoint distance between rangepoints
      breakpoint_max = scan.scan_ratio * scan.points[iter-1].r * (math.sin(angle_diff)/math.sin(0.5*np.pi - angle_diff))
      #print('---')
      #print(l2_norm(scan.points[iter-1], scan.points[iter]))
      #print(breakpoint_max)
        
      if l2_norm(scan.points[iter-1], scan.points[iter]) < breakpoint_max:
        scan_str += "A"
        segment.append(scan.points[iter])
        iter += 1

        if iter == len(scan.points):
          if len(segment) > scan.minsegsize:
            segment_list.append(segment)
          #break

      else:
        scan_str += "-"
        segment_iter = iter
        if len(segment) > scan.minsegsize:
          segment_list.append(segment)
        segment = []
        iter += 1
  
  #print(scan_str) #visualize where segments are
  #print(len(segment_list))
  return segment_list

def detect_circle(segment):
  #print("detected circle: ")
  #print(circle_fit.hyper_fit(np.asarray([ [pt.x, pt.y] for pt in segment])))

  c = Circle()
  c.initialize(circle_fit.hyper_fit(np.asarray([ [pt.x, pt.y] for pt in segment])))
  #return circle_fit.hyper_fit(np.asarray([ [pt.x, pt.y] for pt in segment]))
  return c

  