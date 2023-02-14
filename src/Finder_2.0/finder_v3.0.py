#! /usr/bin/python3
# -*- coding: utf-8 -*-
import sys, time, os

import math, os, os.path, rospy
import numpy as np
from cv_bridge import CvBridge, CvBridgeError
from yolov5_ros_msgs.msg import BoundingBoxes
from darknet_ros_msgs.msg import ObjectCount
from geometry_msgs.msg import *
from math import *
#from people_msgs.msg import *
from scipy.stats import hmean
from sensor_msgs.msg import Image, LaserScan
from sympy import *
from std_msgs.msg import Header, String, ColorRGBA
from time import sleep
from visualization_msgs.msg import MarkerArray, Marker
import yaml
import PIL
from PIL import Image
from sensor_msgs.msg import Image as ImageMsg
from people_msgs.msg import *

from PyQt5.QtCore import *
from PyQt5.QtWidgets import *
from PyQt5.QtGui import *
from PyQt5.Qt import *
from msgs.msg import People, Person

from geometry_msgs.msg import PoseWithCovarianceStamped


from dynamic_reconfigure.parameter_generator_catkin import *

import dynamic_reconfigure.client
from dynamic_reconfigure.server import Server
from people_publisher.cfg import PeoplePublisherConfig
from visualization_msgs.msg import MarkerArray, Marker




exis = Symbol('exis')
bridge = CvBridge()
id_person = 0
shadow_pos_x = []
shadow_pos_y = []


robot_pose = [0, 0, 0]
robot_orientation = [0, 0, 0, 0]
detected_objects = []


class finder_v2dot0:

  def __init__(self):
    #count_sub = rospy.Subscriber('/darknet_ros/found_object', ObjectCount, self.callback_count, queue_size=10)
    
    #emotion_sub = rospy.Subscriber('/facial_emotion', String, self.callback_emotion, queue_size=10)
    #scan_sub = rospy.Subscriber('/scan', LaserScan, self.callback_laser, queue_size=10)
    amcl = rospy.Subscriber('/amcl_pose', PoseWithCovarianceStamped , self.callback_amcl, queue_size=10)
    yolo_sub = rospy.Subscriber('/yolov5/BoundingBoxes', BoundingBoxes , self.callback_box, queue_size=10)
    self.pub_goal = rospy.Publisher('/move_base_simple/goal', PoseStamped, queue_size=10)
    scan_sub = rospy.Subscriber('/scan', LaserScan, self.callback_laser, queue_size=10)
    #count_sub = rospy.Subscriber('/darknet_ros/found_object', ObjectCount, self.callback_count, queue_size=10)



    #self.timer = QTimer(self)
    #self.timer.timeout.connect(self.update)
    #self.timer.start(10)

    self.base_map_dir = os.environ['HOME'] + "/b400Wheelchair_ws/src/smartwheelchair/b400Wheelchair_description/maps/museu.pgm"
    yaml_path = os.environ['HOME'] + "/b400Wheelchair_ws/src/smartwheelchair/b400Wheelchair_description/maps/museu.yaml"
    with open(yaml_path) as f:
        map_data = yaml.safe_load(f)


    self.yaml_resolution = float(map_data["resolution"])
    #print(f"self.yaml_resolution : {self.yaml_resolution}")

    self.yaml_origin = map_data["origin"]
    #print(f"self.yaml_origin : {self.yaml_origin}")

    map_image = PIL.Image.open(self.base_map_dir)
    self.img_data = np.array(map_image)
    self.qimg = QImage(self.img_data.data, self.img_data.shape[1], self.img_data.shape[0], QImage.Format_Indexed8) 
    self.bounds = self.find_bounds(map_image)
    #print(f"bounds : {self.bounds}")

  def callback_amcl(self, data):
    global robot_pose, robot_orientation
    robot_pose = [data.pose.pose.position.x, data.pose.pose.position.y, data.pose.pose.position.z]
    robot_orientation = [data.pose.pose.orientation.x, data.pose.pose.orientation.y, data.pose.pose.orientation.z, data.pose.pose.orientation.w]




  def find_bounds(self, map_image):
      x_min = map_image.size[0]
      x_end = 0
      y_min = map_image.size[1]
      y_end = 0
      pix = map_image.load()
      for x in range(map_image.size[0]):
          for y in range(map_image.size[1]):
              val = pix[x, y]
              if val != 205:
                  x_min = min(x, x_min)
                  x_end = max(x, x_end)
                  y_min = min(y, y_min)
                  y_end = max(y, y_end)
      return x_min, x_end, y_min, y_end


  

  def callback_count(self, msg):
    self.counter = msg.count

  def callback_box(self, data):
    os.system('cls' if os.name == 'nt' else 'clear')
    print('\033[1;30;47m=' * 84)
    print("\033[1;30;47m                                     FINDER v2.1™")
    print('\033[1;30;47m=' * 84)
    print("\n\033[1;34;47mYOLOV3 is detecting {} object(s)...".format(len(data.bounding_boxes)))

    self.lmax_person = []
    self.lmin_person = []
    self.x_center = []
    self.y_center = []
    #self.probab_person = []
    self.classe = []
    self.dist_final = []
    self.dist_x = []
    self.dist_y = []
    self.X_map = []
    self.Y_map = []
    dist = []
    X = []
    Y = []
    Z = []
    Y_robot = []

    X_robot = []
    
    numero = len(data.bounding_boxes) - 1
    #print("({})" .format(numero))

    for f in range(len(data.bounding_boxes)):
      #if (data.bounding_boxes[m].Class) == "person":
        #self.probab_person.append(data.bounding_boxes[f].probability)
        self.classe.append(data.bounding_boxes[f].Class)
        #self.lmin_person.append((((640-data.bounding_boxes[f].xmax)*14)/640))
        #self.lmax_person.append((((640-data.bounding_boxes[f].xmin)*14)/640))

        if (self.classe[f] == "chair"):
          self.x_center.append((data.bounding_boxes[f].xmax + data.bounding_boxes[f].xmin)/2)
          self.y_center.append((data.bounding_boxes[f].ymax - data.bounding_boxes[f].ymin)/4)

        else:
          self.x_center.append((data.bounding_boxes[f].xmax + data.bounding_boxes[f].xmin)/2)
          self.y_center.append((data.bounding_boxes[f].ymax - data.bounding_boxes[f].ymin)/2)



        self.fx = 347.9976
        self.fy = 347.9976
        self.cx = 320
        self.cy = 240

        

        dist.append(depth_frame[int(self.y_center[0]), int(self.x_center[0])])
        X.append(dist[f]*(self.x_center[f] - self.cx)/self.fx)
        Y.append(dist[f]*(self.y_center[f] - self.cy)/self.fy)
        Z.append(dist[f])
        #print("({}, {}, {})" .format(X[f], Y[f], Z[f]))

        # CONTROLA O ANGULO DA CAMERA
        camera_angle = 0.6 # -3.14 a 3.14

        # CALCULA A POSICAO DO ROBO
        X_robot_in_map = (robot_pose[0] - float(self.yaml_origin[0])) / self.yaml_resolution
        Y_robot_in_map = -robot_pose[1] / self.yaml_resolution + (self.bounds[3] + float(self.yaml_origin[1]) / self.yaml_resolution)

        q0 = robot_orientation[0]
        q1 = robot_orientation[1]
        q2 = robot_orientation[2]
        q3 = robot_orientation[3]

        yaw = math.atan2(2*(q0*q1 + q2*q3),(q0**2 - q1**2 - q2**2 + q3**2))
        yaw = -yaw - math.pi/2


        # CONVERTE PARA COORDENADAS DO ROBO
        Y_robot.append(-(X[f] *math.cos(camera_angle) - Z[f] *math.sin(camera_angle)))
        X_robot.append(X[f] *math.sin(camera_angle) + Z[f] *math.cos(camera_angle))
        #print(f"(X_robot, Y_robot) : {X_robot}, {Y_robot}")

        # CONVERTE PARA COORDENADAS DO MAPA
        yaw_robot = -math.pi/2 - yaw

        self.X_map.append(X_robot[f] *math.cos(yaw_robot) - Y_robot[f] *math.sin(yaw_robot) + robot_pose[0])
        self.Y_map.append(X_robot[f] *math.sin(yaw_robot) + Y_robot[f] *math.cos(yaw_robot) + robot_pose[1])
        print(f"(X_robot, Y_robot) : {self.X_map[f]}, {self.Y_map[f]}")

  def callback_laser(self, msg):

    def people_func(nome, X_map, Y_map, xx):

      
      person = Person()
      person.name = nome
      #client = dynamic_reconfigure.client.Client('/people_publisher')

      # posicao objeto no mapa
      person.pose.position.x = X_map
      person.pose.position.y = Y_map
      person.pose.position.z = 0



      person.proxemic.spread.x = xx
      person.proxemic.spread.y = xx
      person.proxemic.spread.z = 0
      person.pose.orientation.w = 1

      person.proxemic.freeBorder = 30
      person.proxemic.lethalBorder = 68


      people.people.append(person)

      #pub.publish(people)
      #rate.sleep()



    def marker_func(object_type, red, green, blue, lmax, lmin, id_type):
      marker = Marker()
      marker.header.stamp = rospy.Time.now()
      marker.header.frame_id = "odom"
      marker.ns = object_type
      marker.type = 3
      marker.action = marker.ADD
      marker.scale.x = 0.50
      marker.scale.y = 0.50
      marker.scale.z = 0.50
      marker.color.a = 0.85
      marker.color.r = red
      marker.color.g = green
      marker.color.b = blue
      marker.lifetime = rospy.Duration(0.30)
      marker.pose.orientation.w = 1.00
      marker.pose.position.x = lmax
      marker.pose.position.y = lmin
      marker.pose.position.z = 0.25
      global id_person
      for o in markerArray.markers:
          o.id = id_type
      markerArray.markers.append(marker)


    #for e in range(len(self.clas_person)):
    marker_pub = rospy.Publisher("/marker_loc", MarkerArray, queue_size=10)
    people_pub = rospy.Publisher("/people", People, queue_size=5)
    rate = rospy.Rate(2) # 10h
    print("TESTE!")

    markerArray = MarkerArray()
    people = People()
    people.header.frame_id = 'map'
    people.header.stamp = rospy.Time.now()

    for l in range(len(self.classe)):
      global id_person
      id_person += 2
      #print("({})" .format(len(self.clas_person)))

      if (self.classe[l] == "person"):
        people_func(self.classe[l], self.X_map[l], self.Y_map[l], 0.7)
        marker_func('object_person', 1.00, 0.00, 0.00, self.X_map[l], self.Y_map[l], id_person)

      if (self.classe[l] == "chair"):
        people_func(self.classe[l], self.X_map[l], self.Y_map[l], 1)
        marker_func('object_person', 1.00, 1.00, 0.00, self.X_map[l], self.Y_map[l], id_person)

      if (self.classe[l] == "vase"):
        people_func(self.classe[l], self.X_map[l], self.Y_map[l], 0.5)
        marker_func('object_person', 0.00, 1.00, 0.00, self.X_map[l], self.Y_map[l], id_person)

      

    #for p in range(len(self.clas_person)):
      #shadow_pos_x.insert(p, ppl.people[p].position.x)
      #shadow_pos_y.insert(p, ppl.people[p].position.y)

    end_time = time.time() + 1
    countTimer = 0.00
    sleepTime = 0.50
    while time.time() < end_time:
      time.sleep(sleepTime)
      people_pub.publish(people)
      marker_pub.publish(markerArray)
      rospy.sleep(0.0001)





def callback_depth(data):
  global depth_frame 
  depth_frame = bridge.imgmsg_to_cv2(data, "passthrough")



if __name__ == '__main__':
  rospy.init_node('finder_alpha')
  os.system('cls' if os.name == 'nt' else 'clear')
  print('\033[1;30;47m=' * 84)
  print("\033[1;30;47m                                     FINDER v2.1™")
  print('\033[1;30;47m=' * 84)
  sleep(1.00)
  
  finder_v2dot0()
  rospy.Subscriber("/camera_link/depth/image_raw", ImageMsg, callback_depth)

  try:
    rospy.spin()
  except KeyboardInterrupt:
    print("Shutting down...")
