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

# Fixing random state for reproducibility
np.random.seed(19680801)

def l2_norm(p1, p2):
  return ( (p1.x - p2.x)**2 + (p1.y - p2.y)**2 )**0.5

def residual_for_circle(point_queue, particle_pt, radius_threshold, radius_variance):
  residual = 0.0
  for pt in point_queue:
    norm = l2_norm(pt, particle_pt)
    if radius_threshold - radius_variance < norm < radius_threshold + radius_variance:
      residual += norm
  return residual


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
      rospy.init_node('laser_listener', anonymous=True)
      rospy.Subscriber("/laser_horizontal_front", LaserScan, self.callback)
      self.pub_random_particles = rospy.Publisher('random_particles', PointCloud, queue_size=10)
      self.pub_chosen_particles = rospy.Publisher('chosen_particles', PointCloud, queue_size=10)
      self.pub_scan_pcd_xy = rospy.Publisher('laser_horizontal_pcd', PointCloud)
      rospy.spin()

  def callback(self, data):
    #print "hello"
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

    particles =  [self.x_max  -self.x_min, self.y_max - self.y_min] * np.random.rand(self.num_particles,2) + [[self.x_min, self.y_min]]
    print(particles.shape)
    particle_pcd = PointCloud()
    #filling pointcloud header
    header = Header()
    header.stamp = rospy.Time.now()
    header.frame_id = 'laser_horizontal_front_link'
    particle_pcd.header = header

    particle_pcd_chosen = PointCloud()
    particle_pcd_chosen.header = header


    point_queue = []
    max_size_point_queue = int(0.5*np.pi/data.angle_increment)
    #filling some points
    for p in particles:
      particle_pt = Point32(p[0], p[1], 0.0)

      visited_particle_pt = [False] * self.num_particles
      particle_pcd.points.append(particle_pt)


    ##for scan_pt in scan_pcd.points:
    ##  #print("norm ")
    ##  point_queue.append(scan_pt)
    ##  if len(point_queue) > max_size_point_queue:
    ##    point_queue.pop(0)
    ##    residual = 0.0
    ##    for i in range(self.num_particles):
    ##      
    ##      particle_pt = Point32(particles[i][0], particles[i][1], 0.3)
    ##      #print(scan_pt)
    ##      #print(particle_pt)
    ##      #print(l2_norm(scan_pt, particle_pt))
    ##      residual = residual_for_circle(point_queue, particle_pt, 0.28, 0.1)
    ##     # if residual > 0.0:
    ##     #   print("residual")
    ##     #   print(residual)
    ##      if 0.01 < residual < 10000 and not visited_particle_pt[i]:
    ##        visited_particle_pt[i] = True
    ##        particle_pcd_chosen.points.append(particle_pt)
    ##        print("residual")
    ##        print(residual)
    ##        print("appended particle")
    ##        print(particle_pt)


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
      scan_pt = Point32(x, y, 0.0)
      scan_pcd.points.append(scan_pt)
      point_queue.append(scan_pt)
      if len(point_queue) > max_size_point_queue:
        point_queue.pop(0)
        residual = 0.0
        for i in range(self.num_particles):
          
          particle_pt = Point32(particles[i][0], particles[i][1], 0.3)
          #print(scan_pt)
          #print(particle_pt)
          #print(l2_norm(scan_pt, particle_pt))
          residual = residual_for_circle(point_queue, particle_pt, 0.28, 0.1)
         # if residual > 0.0:
         #   print("residual")
         #   print(residual)
          if 0.01 < residual < 10000 and not visited_particle_pt[i]:
            visited_particle_pt[i] = True
            particle_pcd_chosen.points.append(particle_pt)
            print("residual")
            print(residual)
            print("appended particle")
            print(particle_pt)
    

    self.pub_scan_pcd_xy.publish(scan_pcd)
    self.pub_chosen_particles.publish(particle_pcd_chosen)
    self.pub_random_particles.publish(particle_pcd)
    
    #x_list = np.asarray(x_list)
    #y_list = np.asarray(y_list)
    #N = len(x_list)
    #colors = np.random.rand(N)
    #area = 5#(30 * np.random.rand(N))**2  # 0 to 15 point radii

    #plt.scatter(x_list, y_list, s=area)#, c=colors, alpha=0.5)
    #plt.show()
    #cv2.imshow('frame',self.frame)
    #cv2.waitKey(1)


if __name__ == '__main__':
    x = Laser2D()
