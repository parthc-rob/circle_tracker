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

import circle_fit

from scipy.stats import norm
########### general functions

def l2_norm(p1, p2):
  return ( (p1.x - p2.x)**2 + (p1.y - p2.y)**2 )**0.5

def l2_norm_xy(x1, y1, x2, y2):
  return ( (x1 - x2)**2 + (y1 - y2)**2 )**0.5

################## class

class TrackedObject:
  def __init__(self):
    self.id = 912
    self.radius = 1.0
    #state
    self.state = np.asarray([[0.0, 0.0, 0.0, 0.0]]).T

    #state uncertainty
    self.P = np.diag([1e-1, 1e-1, 1e-3, 1e-3])
    #time step, update with laserscan
    self.dt = 0.03

    #process model
    self.A = np.asarray([[1.0, 0.0, self.dt, 0.0],
                      [0.0, 1.0, 0.0, self.dt],
                      [0.0, 0.0, 1.0, 0.0],
                      [0.0, 0.0, 0.0, 1.0]])

    #measurement model
    self.H = np.asarray([[1.0, 0.0, 0.0, 0.0],
                        [0.0, 1.0, 0.0, 0.0]])
                             
    #measurement noise
    self.R = np.asarray([[1e-4, 0.0],
                        [0.0, 1e-4]])

    #process noise, assume x,y noise independent
    self.Q = np.asarray([[1e-4, 0.0, 0.0, 0.0],
                      [0.0, 1e-4, 0.0, 0.0],
                      [0.0, 0.0, 5e-3, 0.0],
                      [0.0, 0.0, 0.0, 5e-3]])

  def initialize(self, x, y, id):
    self.state[0] = x 
    self.state[1] = y
    self.id = id

  def update_A(self, dt):
    self.dt = dt
    self.A = np.asarray([[1.0, 0.0, self.dt, 0.0],
                      [0.0, 1.0, 0.0, self.dt],
                      [0.0, 0.0, 1.0, 0.0],
                      [0.0, 0.0, 0.0, 1.0]])

  def measurement_update(self, x, y): 
    S = np.matmul(np.matmul(self.H, self.P), self.H.T) + self.R
    K = np.matmul(np.matmul(self.P, self.H.T), np.linalg.pinv(S))

    #### S = H * P * H.T + R
    #### K = P * H.T * inv(S)
    #### x_dash = Z - H * x
    #### x = x + K * x_dash
    #### 

    #update estimate via measurement
    print("x, y, Z")
    print(x)
    print(y)
    Z = np.array([[x,y]]).T
    print(Z)
    
    #print("H, state shape")
    #print(self.H.shape)
    #print(self.state)
    z_dash = np.matmul(self.H, self.state)
    y = Z - np.matmul(self.H, self.state)
    print(Z.shape)
    print("y, k shape")
    print(y.shape)
    print(self.state)
    self.state = self.state + np.matmul(K, y)
    print(self.state)

    self.P = np.matmul( np.eye(4) - np.matmul(K, self.H) , self.P)
  
  def prediction_update(self, dt):
    self.update_A(dt)
    self.state = np.matmul(self.A, self.state)
    self.P = np.matmul(self.A, np.matmul(self.P, self.A.T)) + self.Q

  def distance_from_state(self, x, y):
    #print("state):")
    #print(self.state)
    #print(self.state[0])

    return l2_norm_xy(self.state[0][0], self.state[1][0], x, y) 
  def get_velocity(self):
    #print(self.state)
    #print(self.state[3])
    #print(float(self.state[3]))

    return (self.state[2]**2 + self.state[3]**2) ** 0.5
