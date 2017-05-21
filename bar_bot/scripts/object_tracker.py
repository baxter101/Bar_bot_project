#!/usr/bin/env python

import cv2
import array
import numpy as np
import cv_bridge
import tf
import rospy
import geometry_msgs.msg
import sensor_msgs.msg
import std_msgs.msg
import imutils
import sensor_msgs.point_cloud2 as pc2
import image_geometry
from math import atan2, pi, sqrt

from roslib import message
from std_msgs.msg import String,Int32
from sensor_msgs.msg import Image,PointCloud2,PointField,CameraInfo
from geometry_msgs.msg import Pose,Point,PoseArray,Quaternion
from cv_bridge import CvBridge,CvBridgeError

import roslib
roslib.load_manifest('camera_tf')

class PoseCalculator():
    def __init__(self):
        self.goal_poses = []
        self.rgb_img = None
        self.pc = None
        self.d_base = []
        self.tf_listener = tf.TransformListener()
        self.camera_model = None
        self.cX = 1
        self.cY = 1
        self.colour = 0
        print "init"

    def subscribe(self):
        # Subscribe to pointcloud from ASUS Xtion
        self.pc_sub = rospy.Subscriber("/camera/depth_registered/points" , PointCloud2 , self.pc_callback)
        # Subscribe to get ASUS Xtion camera info
        self.info_sub = rospy.Subscriber("/camera/rgb/camera_info" , CameraInfo , self.info_callback)
        # Subscribe to get ASUS Xtion RGB image
        self.rgb_sub = rospy.Subscriber("/camera/rgb/image_rect_color", Image , self.rgb_callback)
        # Subscribe to movement node to get target colour
        self.colour_sub = rospy.Subscriber("colour_publisher", Int32 ,self.colour_callback)
        print "subscribed to all nodes"

    def publish(self):
        #Publisher for publishing target pose to movement node
        self.handler_pub = rospy.Publisher("object_tracker" , Pose , queue_size=1)
        self.pub_rate = rospy.Rate(10)

    def info_callback(self , data):
        # Get a camera model object using image_geometry and the camera_info topic
        self.camera_model = image_geometry.PinholeCameraModel()
        self.camera_model.fromCameraInfo(data)
        self.info_sub.unregister()  # Only subscribe once
        print "camera info captured"

    def centroid_callback(self):

        self.goal_poses = []
        self.centroid = (self.cY,self.cX)
        self.pos = self.solve_goal_point(self.centroid)
        theta = atan2(0 , 0)
        quat = tf.transformations.quaternion_from_euler(-pi , 0 , theta)
        self.goal_poses.append(Pose(position=Point(*self.pos) , orientation=Quaternion(*quat)))
        self.object_pose = Pose(position=Point(*self.pos) , orientation=Quaternion(*quat))


    def get_depth(self , x , y):
        gen = pc2.read_points(self.pc , field_names='z' , skip_nans=False , uvs=[(x , y)])
        return next(gen)

    def solve_goal_point(self , centroid):
        # Find the centroid in the point cloud

        # Object Radius to get center of bottle/glass
        radius = 0.03
        x = int(centroid[0])
        y = int(centroid[1])
        depth = self.get_depth(x, y)
        lst = list(depth)
        lst[0] = lst[0] + radius
        depth = tuple(lst)
        # Get direction vector from camera to object
        v = self.camera_model.projectPixelTo3dRay((x , y))

        # Multiply direction vector by depth to get target vector
        d_cam = np.concatenate((depth * np.array(v) , np.ones(1))).reshape((4 , 1))

        #Transform target vector to Baxter base frame
        self.tf_listener.waitForTransform('/base' , '/camera_depth_optical_frame' , rospy.Time() , rospy.Duration(4))
        (trans , rot) = self.tf_listener.lookupTransform('/base' , '/camera_depth_optical_frame' , rospy.Time())

        camera_to_base = tf.transformations.compose_matrix(translate=trans ,
                                                           angles=tf.transformations.euler_from_quaternion(rot))
        self.d_base = np.dot(camera_to_base, d_cam)
        return(self.d_base[0:3])


    def callback1(self , message):
        global xb , yb , obj_color , obj_found , cX , cY , height , width
        xb = 0
        yb = 0
        # Capturing image of camera
        bridge = CvBridge()
        original_img = bridge.imgmsg_to_cv2(message , "bgr8")
        img = bridge.imgmsg_to_cv2(message , "bgr8")
        height , width , depth = img.shape
        cv2.waitKey(1)
        # Converting image to HSV format
        hsv = cv2.cvtColor(img , cv2.COLOR_RGB2HSV)

        if self.colour == 0:
            lower_limit = np.array([110, 140, 140])  # ORANGE cup
            upper_limit = np.array([130, 255, 255])

        if self.colour == 1:
            lower_limit = np.array([13, 50, 50]) # BLUE
            upper_limit = np.array([18, 255, 255])

        if self.colour == 2:
            lower_limit = np.array([145, 50, 50])  # PINK
            upper_limit = np.array([150, 255, 255])

        if self.colour == 3:
            lower_limit = np.array([65,  50, 50]) # GREEN
            upper_limit = np.array([82, 255, 255])

        if self.colour == 4:
            lower_limit = np.array([169, 50, 50])  # PURPLE
            upper_limit = np.array([177, 255, 255])


        # Threshold the HSV image to get only blue colors
        mask = cv2.inRange(hsv, lower_limit, upper_limit)

        kernel = np.ones((20, 20) , np.uint8)
        mask = cv2.erode(mask , kernel , iterations=1)
        mask = cv2.dilate(mask , kernel , iterations=1)

        cnts = cv2.findContours(mask.copy() , cv2.RETR_EXTERNAL , cv2.CHAIN_APPROX_SIMPLE)

        cnts = cnts[0] if imutils.is_cv2() else cnts[1]
        # loop over the contours
        for c in cnts:
            # print cv2.contourArea(c)
            if cv2.contourArea(c) > 1000 and cv2.contourArea(c) < 3000:
                # compute the center of the contour
                M = cv2.moments(c)
                self.cY = int(M["m10"] / M["m00"])
                self.cX = int(M["m01"] / M["m00"])
                area = cv2.contourArea(c)

                # draw the contour and center of the shape on the image
                cv2.drawContours(original_img , [c] , -1 , (0 , 255 , 0) , 2)
                cv2.circle(original_img , (self.cY , self.cX) , 7 , (150 , 150 , 150) , -1)
                cv2.circle(original_img , (323 , 246) , 7 , (150 , 150 , 150) , -1)
                cv2.putText(original_img , "CENTER" , (self.cY - 20 , self.cX - 20) ,
                            cv2.FONT_HERSHEY_SIMPLEX , 0.5 , (255 , 255 , 255) , 2)
                # show the image
                cv2.imshow("Original" , original_img)
        cv2.waitKey(1)
        return

    def pc_callback(self , data):
        self.pc = data
        self.centroid_callback()

    def rgb_callback(self , Image):
        self.rgb_img = Image
        self.callback1(self.rgb_img)

    def colour_callback(self , message):
        self.colour = message.data

def main():
    # Initiate left hand camera object detection node
    rospy.init_node('object_recognition_node')

    # Create names for OpenCV images and orient them appropriately
    pose_calc = PoseCalculator()
    pose_calc.subscribe()
    pose_calc.publish()
    br = tf.TransformBroadcaster()
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        if len(pose_calc.goal_poses) > 0:
            if pose_calc.pos[0][0] > 0 :
                pose_calc.handler_pub.publish(pose_calc.object_pose)
                br.sendTransform((pose_calc.pos[0][0],pose_calc.pos[1][0],pose_calc.pos[2][0]) ,
                                 (0.00 , 0.00 , 0.00 , 1.00) ,
                                 rospy.Time.now() ,
                                 "object_frame" ,
                                 "base")
        rate.sleep()


if __name__ == '__main__':
    main()