#!/usr/bin/env python
import rospy
import numpy as np
import math
import cv2

from sensor_msgs.msg import LaserScan

from sensor_msgs.msg import PointCloud
from geometry_msgs.msg import Point32, Quaternion, Pose, Point, Vector3, PoseStamped, PoseWithCovariance, TwistWithCovariance 
from std_msgs.msg import Header, ColorRGBA
import numpy as np
import matplotlib.pyplot as plt
from visualization_msgs.msg import Marker

from circle_detect import *

# Fixing random state for reproducibility
np.random.seed(19680801)

#def l2_norm(p1, p2):
#  return ( (p1.x - p2.x)**2 + (p1.y - p2.y)**2 )**0.5
#
#def residual_for_circle(point_queue, particle_pt, radius_threshold, radius_variance):
#  residual = 0.0
#  for pt in point_queue:
#    norm = l2_norm(pt, particle_pt)
#    if radius_threshold - radius_variance < norm < radius_threshold + radius_variance:
#      residual += norm
#  return residual

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
      self.marker_lifetime = 0.05 

      self.radius_max = 0.2
      self.radius_min = 0.1
      self.distance_gate = 0.4
      self.velocity_gate = 1.0 
      self.full_velocity = 3.0

      self.objects = []
      self.last_measure_time_sec = None 

      self.last_measurement = []

      rospy.init_node('laser_listener', anonymous=True)
      rospy.Subscriber("/laser_horizontal_front", LaserScan, self.callback)
      self.pub_random_particles = rospy.Publisher('random_particles', PointCloud, queue_size=10)
      self.pub_chosen_particles = rospy.Publisher('chosen_particles', PointCloud, queue_size=10)
      self.pub_scan_pcd_xy = rospy.Publisher('laser_horizontal_pcd', PointCloud)
      self.pub_tracked_object = rospy.Publisher('tracked_object', PoseWithCovariance, queue_size=10)
      self.pub_tracked_object_twist = rospy.Publisher('tracked_object_twist', TwistWithCovariance, queue_size=10)
      self.pub_marker_detected_object = rospy.Publisher('marker_detected_object', Marker, queue_size=10)
      self.pub_marker_tracked_object = rospy.Publisher('marker_tracked_object', Marker, queue_size=10)

      rospy.spin()

  def publish_tracked_circle(self, x, y, radius):
    circle = PoseWithCovariance(
      pose = Pose(
        position = Point(x, y, radius),
        orientation = Quaternion(0, 0, 0, 1)
      )
    )
    #circle.pose.position.x = x
    #circle.pose.position.x = y
    #circle.pose.position.z = radius
    self.pub_tracked_object.publish(circle)

  def publish_circle_marker(self, x, y, radius, marker_id):

    ### Pose Marker
    marker_ = Marker(
        type=Marker.CYLINDER,
        action=Marker.ADD,
        id=marker_id,
        lifetime=rospy.Duration(self.marker_lifetime),
        pose=Pose(Point(x, y, 0.0), Quaternion(0,0,0,1)),
        scale=Vector3(radius, radius, 3.0),
        header=Header(frame_id='laser_horizontal_front_link', stamp = rospy.Time.now()),
        color=ColorRGBA(1.0, 0.2, 0.2, 1.0))
    marker_.ns = "bucket"
    self.pub_marker_detected_object.publish(marker_)
    #print("publishing marker)")

  def publish_detection_marker(self, x, y, radius, marker_id):

    ### Pose Marker
    marker_ = Marker(
        type=Marker.CYLINDER,
        action=Marker.ADD,
        id=marker_id,
        lifetime=rospy.Duration(self.marker_lifetime),
        pose=Pose(Point(x, y, 0.0), Quaternion(0,0,0,1)),
        scale=Vector3(radius, radius, 3.0),
        header=Header(frame_id='laser_horizontal_front_link', stamp = rospy.Time.now()),
        color=ColorRGBA(1.0, 0.2, 0.2, 1.0))
    marker_.ns = "bucket"
    self.pub_marker_tracked_object.publish(marker_)
    #print("publishing marker)")
 
  def publish_track_marker(self, x, y, radius, velocity, marker_id):
    
    vel_intensity = 1.0
    #print(velocity)
    if velocity/self.full_velocity < 1.0: # color code velocity
      vel_intensity = velocity/self.full_velocity

    ### Pose Marker
    marker_ = Marker(
        type=Marker.CYLINDER,
        action=Marker.ADD,
        id=marker_id,
        lifetime=rospy.Duration(1),
        pose=Pose(Point(x, y, 0.0), Quaternion(0,0,0,1)),
        scale=Vector3(radius, radius, 3.0),
        header=Header(frame_id='laser_horizontal_front_link', stamp = rospy.Time.now()),
        color=ColorRGBA(0.2, vel_intensity, 0.2, 1.0))
    marker_.ns = "bucket"
    self.pub_marker_tracked_object.publish(marker_)

    obj_marker_ = Marker(
            type=Marker.TEXT_VIEW_FACING,
            action=Marker.ADD,
            id=marker_id + 99,
            ns="text",
            lifetime=rospy.Duration(self.marker_lifetime),
            pose=Pose(Point(x, y, 5.0), Quaternion(0,0,0,1)),
            scale = Vector3(4, 4, 40.0),
            text= "x,y: ", #+ str(x) + ", "+str(y) + " vel(m/s): "+str(velocity),
            header=Header(frame_id='laser_horizontal_front_link'),
            color=ColorRGBA(0.0, 1.0, 0.0, 1))

    self.pub_marker_tracked_object.publish(marker_)
#  ## velocity arrow
#    marker_ = Marker(
#        type=Marker.ARROW,
#        action=Marker.ADD,
#        id=marker_id,
#        lifetime=rospy.Duration(self.marker_lifetime),
#        pose=Pose(Point(x, y, 0.0), Quaternion(0,0,0,1)),
#        scale=Vector3(radius, radius, 3.0),
#          header=Header(frame_id='laser_horizontal_front_link', stamp = rospy.Time.now()),
#          color=ColorRGBA(1.0, 0.2, 0.2, 1.0))
#      marker_.ns = "bucket"
#      self.pub_marker_tracked_object.publish(marker_)

  def callback(self, data):

    segments = get_segments(preprocess_scan(data)) 
    random_id = 920
    z_list = [] # list of z : measurments for cylindrical objects 

    for s in segments:
      c = detect_circle(s)
      #print("circle x,y,radius : %f, %f, %f" %(c.a, c.b, c.radius))

      if c.radius < self.radius_max and c.radius > self.radius_min:
        z_list.append(c)
        #print("BUCKET DETECTED at : %f, %f, %f" %(c.a, c.b, c.radius))
        self.publish_circle_marker(c.a, c.b, c.radius, random_id)
        self.publish_tracked_circle(c.a, c.b, c.radius)
        #self.publish_marker(c.a, c.b, random_id)
      random_id += 1

    random_id = 0 
    dt = 0.03 
    if self.last_measure_time_sec is not None:
      dt = rospy.get_time() - self.last_measure_time_sec
    self.last_measure_time_sec = rospy.get_time()

    ## ## Approach 1
    ##print("measurement list")
    ##print(len(z_list))

    if len(self.last_measurement) > 0:
      for z in z_list:
        for z_old in self.last_measurement:
          if l2_norm_xy(z.a, z.b, z_old.a, z_old.b) < self.distance_gate:
            velocity =  l2_norm_xy(z.a, z.b, z_old.a, z_old.b)/dt
            #self.publish_track_marker(z.a, z.b, z.radius,
            #    velocity, np.random.randint(999))
            self.publish_track_marker(z.a, z.b, z.radius,
                velocity, np.random.randint(999))
            if velocity > self.velocity_gate: # only print for moving objects
              print( " Object at ( %.2f, %.2f ) has velocity %.2f m/s"%(z.a, z.b,velocity))


    self.last_measurement = z_list   

    ## Kalman filter based approach
    if len(self.objects) == 0:
      for measure in z_list:
        #print("init track: " + str(random_id))
        obj = TrackedObject()
        obj.initialize(measure.a, measure.b, random_id)
        obj.update_A(dt)
        random_id += 1
        self.objects.append(obj)
    
    else:
      # quick and dirty distance gating-based matching 
      
      for measure in z_list:
        associated = False
        for obj in self.objects:
          if associated:
            break
          if obj.distance_from_state(measure.a, measure.b) < 1.0:
            #print("associate xy (%.2f, %.2f) with ID %d at xy (%.2f, %.2f) at distance %.2f"
            #      %(measure.a, measure.b, obj.id, obj.state[0][0], obj.state[1][0], obj.distance_from_state(measure.a, measure.b)))

            #print(obj.distance_from_state(measure.a, measure.b))
            obj.measurement_update(measure.a, measure.b)
            obj.radius = measure.radius
            #### Show Kalman track
            #self.publish_track_marker(obj.state[0][0], obj.state[1][0], obj.radius, obj.get_velocity(), obj.id)           

            obj.prediction_update(dt)
            associated = True 

        if not associated:
          random_id = np.random.randint(999)
          #print("new track: " + str(random_id) + " at xy " + str(measure.a) + ", " + str(measure.b))
          obj = TrackedObject()
          obj.initialize(measure.a, measure.b, random_id)
          obj.radius = measure.radius
          obj.update_A(dt)
          self.objects.append(obj)
          #### Show Kalman track

          #self.publish_track_marker(obj.state[0][0], obj.state[1][0], obj.radius, obj.get_velocity(), obj.id)

    for obj in self.objects:
      obj.prediction_update(dt)

if __name__ == '__main__':
    x = Laser2D()
