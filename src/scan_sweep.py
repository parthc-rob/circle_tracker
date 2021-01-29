#!/usr/bin/env python
import rospy
import numpy as np
import math
import cv2

from sensor_msgs.msg import LaserScan

from sensor_msgs.msg import PointCloud
from geometry_msgs.msg import Point32, Twist, Pose, PoseWithCovariance, TwistWithCovariance
from std_msgs.msg import Header
from visualization_msgs.msg import Marker
import numpy as np
import matplotlib.pyplot as plt

from scipy import optimize

from sklearn.cluster import KMeans

# Fixing random state for reproducibility
np.random.seed(19680801)

class State:
  pose = PoseWithCovariance()
  twist = TwistWithCovariance()
  header = Header() 

class PointQueue:
  def __init__(self, point_sweep_queue_size_):
    self.points = []
    self.point_sweep_queue_size = point_sweep_queue_size_

  def add_point(self, p): #Point32
    if len(self.points) > self.point_sweep_queue_size :
      self.points.pop(0)
    self.points.append(p)

  def is_full(self):
    if self.point_sweep_queue_size is None:
      return False
    else:
      if len(self.points) < self.point_sweep_queue_size:
        return True
      return False
  
  def check_state_proposal(self, proposal): #State
    print("checking proposal")
  def calc_R(self, x, y, xc, yc):
        """ calculate the distance of each 2D points from the center (xc, yc) """
        return np.sqrt((x-xc)**2 + (y-yc)**2)

  def f_2(self, c, x, y):
      """ calculate the algebraic distance between the data points and the mean circle centered at c=(xc, yc) """
      Ri = self.calc_R(x,y, *c)
      return Ri - Ri.mean()

  def find_clusters(self): #kmeans 
    if len(self.points) < 4:
      return
    X = np.array( [[pt.x, pt.y] for pt in self.points[:] ])

    kmeans = KMeans(n_clusters=4)
    kmeans.fit(X)
    y_kmeans = kmeans.predict(X)
    centers = kmeans.cluster_centers_

    center_estimate = self.points[ len(self.points)/2 ].x, self.points[ len(self.points)/2 ].y
    print("\nCenter estimate: ")
    print(center_estimate)

    print(" \n kmeans clusters: ")
    print(centers)
    return centers

  def fit_circle(self):
    print("fitting circle")
    center_estimate = self.points[ len(self.points)/2 ].x, self.points[ len(self.points)/2 ].y 
    #center_2, ier = optimize.leastsq(f_2, center_estimate)

    #xc_2, yc_2 = center_2
    #Ri_2       = calc_R(, *center_2)
    #R_2        = Ri_2.mean()
    #residu_2   = sum((Ri_2 - R_2)**2)
    #print("\nCenter estimate: ")
    #print(center_estimate)
    #print("\ncenter_leastsq: ")
    #print(center_2)
    #return center_2
  
    ####
    x = np.array( [pt.x for pt in self.points[:] ])
    y = np.array( [pt.y for pt in self.points[:] ])

    ###
    #x=np.array([1.0,2.5,3.5,4.0,1.1,1.8,2.2,3.7])
    #y=np.array([6.008,15.722,27.130,33.772,5.257,9.549,11.098,28.828])
   # here, create lambda functions for Line, Quadratic fit
   # tpl is a tuple that contains the parameters of the fit



   #funcLine=lambda tpl,x : tpl[0]*x+tpl[1]
   #funcQuad=lambda tpl,x : tpl[0]*x**2+tpl[1]*x+tpl[2]
   # func is going to be a placeholder for funcLine,funcQuad or whatever 
   # function we would like to fit
   #func=funcLine
   # ErrorFunc is the diference between the func and the y "experimental" data
   #ErrorFunc=lambda tpl,x,y: func(tpl,x)-y
   #tplInitial contains the "first guess" of the parameters 
   #tplInitial1=(1.0,2.0)
   # leastsq finds the set of parameters in the tuple tpl that minimizes
   # ErrorFunc=yfit-yExperimental
    center_2,success = optimize.leastsq(self.f_2, center_estimate, args=(x,y))
    #print " linear fit ",tplFinal1
    #xx1=np.linspace(x.min(),x.max(),50)
    #yy1=func(tplFinal1,xx1)
    print("\nCenter estimate: ")
    print(center_estimate)
    print("\ncenter_leastsq: ")
    print(center_2)
    return center_2
  



class Laser2D:

  def __init__(self):
      self.frame = np.zeros((500, 500,3), np.uint8)
      self.x_range = [0.0, 0.0]
      self.y_range = [0.0, 0.0]
      self.x_max = 9#-10000.0
      self.y_max = 18#-10000.0
      self.x_min = -4#99999.0
      self.y_min = -4#99999.0

      self.num_particles = 100
      self.point_sweep_queue_size = None
      self.point_sweep_queue = None

      self.marker_lifetime = 3

      rospy.init_node('laser_listener', anonymous=True)

      self.state_proposal = State()
      self.state_proposal.header.stamp = rospy.Time.now()

      rospy.Subscriber("/laser_horizontal_front", LaserScan, self.callback)

      self.pub_random_particles = rospy.Publisher('random_particles', PointCloud, queue_size=10)
      self.pub_scan_pcd_xy = rospy.Publisher('laser_horizontal_pcd', PointCloud)
      self.pub_tracked_object = rospy.Publisher('tracked_object', PoseWithCovariance, queue_size=10)
      self.pub_tracked_object_twist = rospy.Publisher('tracked_object_twist', TwistWithCovariance, queue_size=10)
      self.pub_marker_tracked_object = rospy.Publisher('marker_tracked_object', Marker, queue_size=10)

      rospy.spin()

  def publish_marker(self, x, y, id):

    marker_ = Marker()
    header = Header()
    header.stamp = rospy.Time.now()
    header.frame_id = 'laser_horizontal_front_link'
 
    marker_.header = header
    marker_.type = marker_.CYLINDER
    marker_.action = marker_.ADD
    marker_.id = id 

    marker_.pose.position.x = x #pos_[0]
    marker_.pose.position.y = y #pos_[1]
    marker_.pose.position.z = 0.0 
    #marker_.pose.orientation.x = ori_[1]
    #marker_.pose.orientation.y = ori_[2]
    #marker_.pose.orientation.z = ori_[3]
    #marker_.pose.orientation.w = ori_[0]

    marker_.lifetime = rospy.Duration.from_sec(self.marker_lifetime)
    marker_.scale.x = 0.28#scale_[0]
    marker_.scale.y = 0.28#scale_[1]
    marker_.scale.z = 1.0#scale_[2]
    marker_.color.a = 0.5
    #red_, green_, blue_ = color_
    marker_.color.r = 1.0#red_
    marker_.color.g = 0.0#green_
    marker_.color.b = 0.0#blue_  
    self.pub_marker_tracked_object.publish(marker_)

  def callback(self, data):
    #print "hello"

    if self.point_sweep_queue_size is None:
      self.point_sweep_queue_size = 0.5*np.pi / data.angle_increment
      self.point_sweep_queue = PointQueue(self.point_sweep_queue_size)

    self.frame = np.zeros((500, 500,3), np.uint8)
    angle = data.angle_min
    x_list = []
    y_list = []
    scan_pcd = PointCloud()
    #filling pointcloud header
    header = Header()
    header.stamp = rospy.Time.now()
    header.frame_id = 'laser_horizontal_front_link'
    scan_pcd.header = header
    #filling some points
    
    for r in data.ranges:
      #change infinite values to 0
      if math.isinf(r) == True:
        r = 0
      x = (r)*math.cos(angle)#+(-90.0*3.1416/180.0)))                         
      y = (r)*math.sin(angle)# + (-90.0*3.1416/180.0)))
      boundary = 500
      if y > boundary or y < -boundary or x<-boundary or x>boundary:
          x=0
          y=0
      #cv2.line(self.frame,(250, 250),(x+250,y+250),(255,0,0),2)
      #x_list.append(angle)
      #y_list.append(r)
      #x_list.append(x)
      #y_list.append(y)
      angle= angle + data.angle_increment
      #cv2.circle(self.frame, (250, 250), 2, (255, 255, 0))
     # if x > self.x_max:
     #   self.x_max = x
     # if y > self.y_max:
     #   self.y_max = x
     # if x < self.x_min:
     #   self.x_min = x
     # if y < self.y_min:
     #   self.y_min = y
      scan_pcd.points.append(Point32(x, y, 0.0))

      #### add points to queue
      self.point_sweep_queue.add_point(Point32(x, y, 0.0))
      if self.point_sweep_queue.is_full():
        center_estim = self.point_sweep_queue.find_clusters()
        init_id = 22
        if center_estim is not None:
          for center in center_estim:
            self.publish_marker(center[0], center[1], init_id)
            init_id = init_id + 1



    self.pub_scan_pcd_xy.publish(scan_pcd)
    
    particles =  [self.x_max  -self.x_min, self.y_max - self.y_min] * np.random.rand(self.num_particles,2) + [[self.x_min, self.y_min]]
    print(particles.shape)
    particle_pcd = PointCloud()
    #filling pointcloud header
    header = Header()
    header.stamp = rospy.Time.now()
    header.frame_id = 'laser_horizontal_front_link'
    particle_pcd.header = header
    #filling some points
    for p in particles:
      particle_pcd.points.append(Point32(p[0], p[1], 0.0))
    self.pub_random_particles.publish(particle_pcd)
    print("\nXrange: " + str(self.x_min) + "," + str(self.x_max))
    print("\nYrange: " + str(self.y_min) + "," + str(self.y_max))

    #x_list = np.asarray(x_list)
    #y_list = np.asarray(y_list)
    #N = len(x_list)
    #colors = np.random.rand(N)
    #area = 5#(30 * np.random.rand(N))**2  # 0 to 15 point radii

    #plt.scatter(x_list, y_list, s=area)#, c=colors, alpha=0.5)
    #plt.show()
    #cv2.imshow('frame',self.frame)
    #cv2.waitKey(1)



    marker_ = Marker()
    marker_.header = header
    marker_.type = marker_.CYLINDER
    marker_.action = marker_.ADD
    marker_.id = 999

    marker_.pose.position.x = self.state_proposal.pose.pose.position.x #pos_[0]
    marker_.pose.position.y = self.state_proposal.pose.pose.position.y #pos_[1]
    marker_.pose.position.z = 0.0 
    #marker_.pose.orientation.x = ori_[1]
    #marker_.pose.orientation.y = ori_[2]
    #marker_.pose.orientation.z = ori_[3]
    #marker_.pose.orientation.w = ori_[0]

    marker_.lifetime = rospy.Duration.from_sec(self.marker_lifetime)
    marker_.scale.x = 0.28#scale_[0]
    marker_.scale.y = 0.28#scale_[1]
    marker_.scale.z = 1.0#scale_[2]
    marker_.color.a = 0.5
    #red_, green_, blue_ = color_
    marker_.color.r = 1.0#red_
    marker_.color.g = 0.0#green_
    marker_.color.b = 0.0#blue_  
    self.pub_marker_tracked_object.publish(marker_)

if __name__ == '__main__':
    x = Laser2D()
