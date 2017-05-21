#!/usr/bin/env python

import sys
import copy
import rospy
import cv2
import cv_bridge
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
import baxter_interface
from baxter_interface import Gripper
import numpy
from tf import transformations as t

from moveit_msgs.msg import ExecuteTrajectoryActionResult
from std_msgs.msg import (Header, String, Int32 , Bool)
from geometry_msgs.msg import (PoseStamped, Pose, Point, Quaternion)
from baxter_core_msgs.msg import (EndEffectorState,EndpointState,HeadPanCommand,NavigatorState)

from sensor_msgs.msg import Image
from moveit_commander import MoveGroupCommander

from geometry_msgs.msg import Pose, PoseStamped

from baxter_core_msgs.srv import (SolvePositionIK,
                                  SolvePositionIKRequest)

def init():
    # Wake up Baxter
    baxter_interface.RobotEnable().enable()
    rospy.sleep(0.25)
    print "Baxter is enabled"

    #Display initialization screen
    display_publsisher("/home/ubuntuz/ws_ros/src/bar_bot2/images/Init.png" , 0.0)

    print "Intitializing clients for services"
    global ik_service_left
    ik_service_left = rospy.ServiceProxy(
        "ExternalTools/left/PositionKinematicsNode/IKService",
        SolvePositionIK)

    global ik_service_right
    ik_service_right = rospy.ServiceProxy(
        "ExternalTools/right/PositionKinematicsNode/IKService",
        SolvePositionIK)

    # Taken from the MoveIt Tutorials
    moveit_commander.roscpp_initialize(sys.argv)
    robot = moveit_commander.RobotCommander()

    # Activate Left Arm to be used with MoveIt
    global left_group
    left_group = MoveGroupCommander("left_arm")
    left_group.set_goal_position_tolerance(0.01)
    left_group.set_goal_orientation_tolerance(0.01)
    
    # Activate and calibrate left gripper
    global left_gripper
    left_gripper = baxter_interface.Gripper('left')
    left_gripper.calibrate()
    print left_gripper.parameters()
    
    #Subscribe to get end effector state
    global endpoint_subs
    endpoint_subs = rospy.Subscriber("/robot/limb/left/endpoint_state", EndpointState, end_point_callback)
    rospy.sleep(1)
    
    #Subscribe to external camera object tracker node
    global object_tracker_subs
    object_tracker_subs = rospy.Subscriber("/object_tracker", Pose, tracker_callback)
    rospy.sleep(1)
    
    #Subscribe to offset tracker node 
    global offset_tracker_subs
    offset_tracker_subs = rospy.Subscriber("/arm_offset_tracker", Point, offset_callback)
    rospy.sleep(1)
    
    #Subscribe to right arm navigator 
    global navigator_sub
    navigator_sub = rospy.Subscriber("/robot/navigators/right_navigator/state", NavigatorState, navigator_callback)
    rospy.sleep(1)

# Callback functions
def tracker_callback(msg):
    global object_pose
    object_pose = msg

def offset_callback(msg):
    global offset_point
    offset_point = msg

def navigator_callback(msg):
    global navigator
    navigator = msg

def publish(target_colour):
    handler_pub = rospy.Publisher("colour_publisher" , Int32 , queue_size=1)
    handler_pub.publish(target_colour)

def request_pose(pose, arm, groupl):
    # Function for planning and execution of end-effector movement in cartesian tool space
    # Set stamped pose
    pose_stamped = PoseStamped()
    pose_stamped.pose = pose
    pose_stamped.header.frame_id = "base"
    pose_stamped.header.stamp = rospy.Time.now()

    # Create IK request
    ik_request = SolvePositionIKRequest()
    ik_request.pose_stamp.append(pose_stamped)

    arm_name_service = "ExternalTools/" + arm + "/PositionKinematicsNode/IKService"
    # Request service
    try:
        rospy.wait_for_service(arm_name_service, 5.0)
        if arm == "left":
            ik_response = ik_service_left(ik_request)
        else:
            ik_response = ik_service_right(ik_request)
    except (rospy.ServiceException, rospy.ROSException), error_message:
        rospy.logerr("Service request failed: %r" % (error_message))
        sys.exit("ERROR - move_to_observe - Failed to append pose")
    if ik_response.isValid[0]:
        limb_joints = dict(zip(ik_response.joints[0].name, ik_response.joints[0].position))
        print "Valid IK response , Printing Joint limbs : ";
        print limb_joints;
        groupl.set_start_state_to_current_state()
        groupl.set_joint_value_target(limb_joints)
        plan2 = groupl.plan(limb_joints)
        rospy.sleep(2)
        groupl.execute(plan2)
        rospy.sleep(4)
        return True

    else:
        return False

def move_to_start_position():

    print "Moving to start"
    start_pose = Pose()
    start_pose.position = Point(0.950,0.723,0.374)
    start_pose.orientation = Quaternion(2.00, 0.00, 1.00, 0.00)
    request_pose(start_pose, "left", left_group)
    return True

def serve_drink():
    global z_force , x_force
    print "Moving to service position A"
    start_pose = Pose()
    start_pose.position = Point(0.900,0.623,0.124)
    start_pose.orientation = Quaternion(0.00, 1.00, 0.00, 1.00)
    request_pose(start_pose, "left", left_group)
    
    print "Moving to service position B"
    start_pose.position = Point(0.970,0.623, 0.174)
    request_pose(start_pose, "left", left_group)
 
    display_publsisher("/home/ubuntuz/ws_ros/src/bar_bot2/images/Drink.png" , 0.5)
    rospy.sleep(1)
    x_force = endpoint_state.x
    z_force = endpoint_state.z

    while  endpoint_state.x > x_force -2 and endpoint_state.z > z_force - 2  :
        rospy.sleep(0.01)
        print "waiting on pickup"

    left_gripper.open()
    display_publsisher("/home/ubuntuz/ws_ros/src/bar_bot2/images/Success.png" , 0.5)
    rospy.sleep(1)
    print "Moving to service position C"
    start_pose.position = Point(0.870, 0.623, 0.174)
    request_pose(start_pose, "left", left_group)

def pickup_glass():

    print "Pick up glass A"
    global current_pose , target_pose , target_pose2
    current_pose = Pose()
    current_pose.position = Point(object_pose.position.x - 0.07,object_pose.position.y ,object_pose.position.z)
    current_pose.orientation = Quaternion(0.00, 1.00, 0.00, 1.00)
    #Move to Point C
    request_pose(current_pose, "left", left_group)

    print "Pick up glass B"
    target_pose = Pose()

    rospy.sleep(4)
    print offset_point.x
    target_pose.position = Point(current_pose.position.x + offset_point.x,current_pose.position.y + offset_point.y,current_pose.position.z +offset_point.z)
    target_pose.orientation = Quaternion(0.00, 1.00, 0.00, 1.00)
    
    #Move to Point B
    request_pose(target_pose, "left", left_group)

    now = rospy.get_time()
    while rospy.get_time() < now + 3:
        print "waiting on offset"

    left_gripper.close()
    rospy.sleep(1)
    print "Pick up glass C"
    target_pose2 = Pose()
    target_pose2.position = Point(target_pose.position.x -0.14 ,target_pose.position.y ,target_pose.position.z+ 0.15)
    target_pose2.orientation = Quaternion(0.00, 1.00, 0.00, 1.00)

    #Move to Point C
    request_pose(target_pose2, "left", left_group)

def dispense_drink():
    print "Starting to dispense"
    global current_pose , target_pose , target_pose2
    current_pose = Pose()
    current_pose.position = Point(object_pose.position.x + 0.040,object_pose.position.y-0.05 ,object_pose.position.z -0.38 )
    current_pose.orientation = Quaternion(0.00, 1.00, 0.00, 1.00)
    print "Move to dispense A"
    request_pose(current_pose, "left", left_group)

    print "Move to dispense B"
    target_pose = Pose()
    target_pose.position = Point(current_pose.position.x  ,current_pose.position.y,current_pose.position.z +0.06 )
    target_pose.orientation = Quaternion(0.00, 1.00, 0.00, 1.00)
    request_pose(target_pose, "left", left_group)

    print "Move to dispense C"
    target_pose.position = Point(target_pose.position.x, target_pose.position.y,target_pose.position.z - 0.06)
    request_pose(target_pose, "left", left_group)
    return True

def end_point_callback(msg):
    global endpoint_state
    endpoint_state = EndpointState()
    endpoint_state = msg.wrench.force

def display_publsisher(path , pan_target):
    img = cv2.imread(path)
    msg = cv_bridge.CvBridge().cv2_to_imgmsg(img, encoding="bgr8")
    pub = rospy.Publisher('/robot/xdisplay', Image, latch=True, queue_size=1)
    pub.publish(msg)
    rospy.sleep(1)
    for i in range (1,10):
        pub3 = rospy.Publisher('/robot/head/command_head_pan', HeadPanCommand, queue_size=1)
        msg = HeadPanCommand()
        msg.target = pan_target
        msg.speed_ratio = 100
        msg.enable_pan_request = True
        pub3.publish(msg)
        # Sleep to allow for image to be published.
        rospy.sleep(0.1)

def main():

    print "==============Main code starts here====================="
    rospy.init_node('baxter_mover_node')
    print "Initializing all MoveIt related functions and services"
    init()

    #Navigator selection statuses()
    switch1 = True
    switch2 = True
    switch3 = True
    switch4 = True
    switch5 = True

    move_to_start_position()
    left_gripper.open()
    
    # Wait for navigator wheel to be used
    wheel_start = navigator.wheel
    display_publsisher("/home/ubuntuz/ws_ros/src/bar_bot2/images/OK.png", -0.5)
    while wheel_start == navigator.wheel :
        rospy.sleep(0.01)

    #Drink selection stage
    while navigator.buttons[0] is False:
        if navigator.wheel > 0 and navigator.wheel < 12 or navigator.wheel > 59 and navigator.wheel < 73 or navigator.wheel > 121 and navigator.wheel < 136 or navigator.wheel > 184 and navigator.wheel < 202:
            if switch1 == True :
                display_publsisher("/home/ubuntuz/ws_ros/src/bar_bot2/images/OrangeLong.png", -0.5)
                switch1 = False
                switch2 = True
                switch3 = True
                switch4 = True
                switch5 = True
                drink_id = 0
            recipe = [4, 4, 1, 2, 3]

        if navigator.wheel > 11 and navigator.wheel < 24 or navigator.wheel > 72 and navigator.wheel < 85 or navigator.wheel > 135 and navigator.wheel < 148 or navigator.wheel > 201 and navigator.wheel < 218:
            if switch2 == True :
                display_publsisher("/home/ubuntuz/ws_ros/src/bar_bot2/images/RumScrew.png", -0.5)
                switch2 = False
                switch1 = True
                switch3 = True
                switch4 = True
                switch5 = True
                drink_id = 1
            recipe = [4, 4, 2]


        if navigator.wheel > 23 and navigator.wheel < 36 or navigator.wheel > 84 and navigator.wheel < 97 or navigator.wheel > 147 and navigator.wheel < 161 or navigator.wheel > 217 and navigator.wheel < 232:
            if switch3 == True :
                display_publsisher("/home/ubuntuz/ws_ros/src/bar_bot2/images/ScrewDriver.png", -0.5)
                switch3 = False
                switch2 = True
                switch1 = True
                switch4 = True
                switch5 = True
                drink_id = 2
            recipe = [4, 4, 1]


        if navigator.wheel > 35 and navigator.wheel < 48 or navigator.wheel > 96 and navigator.wheel < 110 or navigator.wheel > 160 and navigator.wheel < 173 or navigator.wheel > 231 and navigator.wheel < 242:
            if switch4 == True :
                display_publsisher("/home/ubuntuz/ws_ros/src/bar_bot2/images/TequilaSun.png", -0.5)
                switch4 = False
                switch2 = True
                switch3 = True
                switch1 = True
                switch5 = True
                drink_id = 3
            recipe = [4, 4, 3]

        if navigator.wheel > 47 and navigator.wheel < 60 or navigator.wheel > 109 and navigator.wheel < 122 or navigator.wheel > 172 and navigator.wheel < 185 or navigator.wheel > 241 and navigator.wheel < 256:
            if switch5 == True :
                display_publsisher("/home/ubuntuz/ws_ros/src/bar_bot2/images/TequilaSurprise.png", -0.5)
                switch5 = False
                switch2 = True
                switch3 = True
                switch4 = True
                switch1 = True
                drink_id = 4
            recipe = [4, 4, 2, 3]

    #The second pictures showing what you have ordered
    drink_pics =["/home/ubuntuz/ws_ros/src/bar_bot2/images/Long island.png",
                "/home/ubuntuz/ws_ros/src/bar_bot2/images/Rum screw driver.png",
                "/home/ubuntuz/ws_ros/src/bar_bot2/images/Screw driver.png",
                "/home/ubuntuz/ws_ros/src/bar_bot2/images/Tequila sun.png",
                "/home/ubuntuz/ws_ros/src/bar_bot2/images/Tequila surprise.png"]

    display_publsisher(drink_pics[drink_id],-0.5)

    # Pick up glass
    pickup_glass()

    # Display next ingredient
    for j in range (0,len(recipe)) :
        if recipe[j] == 1:
            display_publsisher("/home/ubuntuz/ws_ros/src/bar_bot2/images/Dispensing vodka.png", 0.0)
        if recipe[j] == 4:
            display_publsisher("/home/ubuntuz/ws_ros/src/bar_bot2/images/Dispensing orange juice.png", 0.0)
        if recipe[j] == 2:
            display_publsisher("/home/ubuntuz/ws_ros/src/bar_bot2/images/Dispensing rum.png", 0.0)
        if recipe[j] == 3:
            display_publsisher("/home/ubuntuz/ws_ros/src/bar_bot2/images/Dispensing tequila.png", 0.0)

        #Publish next colour to look for
        now = rospy.get_time()
        while rospy.get_time() < now + 3:
            publish(recipe[j])
        dispense_drink()

    #Serve drink
    serve_drink()

    # Move to start poisition
    move_to_start_position()

if __name__ == '__main__':
    main()
