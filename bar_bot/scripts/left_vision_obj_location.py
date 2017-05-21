#!/usr/bin/env python
#Reference: the baxter_bar-bot project by students in Aalborg University Copengagen MOE program

import rospy
import numpy as np
import cv2
import cv_bridge
import baxter_interface
import imutils
import image_geometry
import tf


from std_msgs.msg import String, Int32
from sensor_msgs.msg import (Image,CameraInfo)
from geometry_msgs.msg import Point
from baxter_core_msgs.msg import AnalogIOState


from cv_bridge import CvBridge, CvBridgeError

#Object centroid position in the baxter's stationary base frame 

def callback(message):

    global  d_cam

    #Capturing image of camera
    bridge = CvBridge()
    cv_image = bridge.imgmsg_to_cv2(message, "bgr8")

    #Converting image to HSV format
    hsv = cv2.cvtColor(cv_image, cv2.COLOR_RGB2HSV)

        # Analyze image for green objects

    lower_limit = np.array([112, 50, 50])  # ORANGE cup
    upper_limit = np.array([128, 255, 255])
    mask = cv2.inRange(hsv, lower_limit, upper_limit)

    kernel = np.ones((20, 20) , np.uint8)
    mask = cv2.erode(mask , kernel , iterations=1)
    mask = cv2.dilate(mask , kernel , iterations=1)

    cnts = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    cnts = cnts[0] if imutils.is_cv2() else cnts[1]
    # loop over the contours
    for c in cnts:
        # print cv2.contourArea(c)
        if cv2.contourArea(c) > 1000 :
            # compute the center of the contour
            M = cv2.moments(c)
            cY = int(M["m10"] / M["m00"])
            cX = int(M["m01"] / M["m00"])
            cv2.contourArea(c)
            # draw the contour and center of the shape on the image
            cv2.drawContours(cv_image, [c], -1, (0, 255, 0), 2)
            cv2.circle(cv_image, (cY, cX), 7, (150, 150, 150), -1)
            cv2.circle(cv_image, (323, 246), 7, (150, 150, 150), -1)
            cv2.putText(cv_image, "CENTER", (cY - 20, cX - 20),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)
            cv2.imshow("Original", cv_image)
            depth = 0.07
            v = camera_model.projectPixelTo3dRay((cX, cY))
            d_cam = np.concatenate((depth * np.array(v), np.ones(1))).reshape((4, 1))
            offset_publisher()

    cv2.waitKey(1)

    return

def range_callback(message):
    global IR_range
    IR_range = message.value

def offset_publisher():
    handler_pub = rospy.Publisher("arm_offset_tracker", Point, queue_size=1)
    offset = Point()
    # Constants added from quality control testing
    offset.z = d_cam[0] - 0.05875
    offset.x = d_cam[2] + 0.11
    offset.y = d_cam[1]
    handler_pub.publish(offset)

def info_callback(data):
    global camera_model
    # Get a camera model object using image_geometry and the camera_info topic
    camera_model = image_geometry.PinholeCameraModel()
    camera_model.fromCameraInfo(data)
    info_sub.unregister()  # Only subscribe once

def main():
    global info_sub , br
    #Initiate left hand camera object detection node
    rospy.init_node('left_camera_node')
    br = tf.TransformBroadcaster()
    info_sub = rospy.Subscriber("/cameras/left_hand_camera/camera_info_std", CameraInfo, info_callback)
    #Subscribe to left hand camera image 
    rospy.Subscriber("/cameras/left_hand_camera/image", Image, callback)
    rospy.Subscriber("/robot/analog_io/left_hand_range/state", AnalogIOState , range_callback)
    rospy.spin()


if __name__ == '__main__':
     main()
