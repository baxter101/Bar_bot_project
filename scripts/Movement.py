import baxter_interface
import baxter_external_devices
import rospy
import ik_solver
from geometry_msgs.msg import (
    Point,
    Quaternion,
)


# Moves relative to current Position
def moveRel(x,y,z):
	pose = left.endpoint_pose()
	print "current pose: ", pose
	curX = pose["position"].x
	curY = pose["position"].y
	curZ = pose["position"].z
	newX = curX + x
	newY = curY + y
	newZ = curZ + z
	loc = Point(newX,newY,newZ)
	print "loc: ", loc
	print "orient: ", faceDown
	limb_joints = ik_solver.ik_solve('left', loc, faceDown)
	prev = loc
	if limb_joints != -1:
	    print "limb_joints: ", limb_joints
	    print "moving arm to limb_joints joints"
	    left.move_to_joint_positions(limb_joints)


# Takes a Point()
def moveTo(goal):
	loc = goal
	#print "loc: ", loc
	#print "orient: ", orient[1]
	limb_joints = ik_solver.ik_solve('left', loc, faceDown)
	prev = loc
	if limb_joints != -1:
	    #print "limb_joints: ", limb_joints
	    #print "moving arm to goal limb_joints joints"
	    print "  "
	    print "  "
	    print "  "
	    left.move_to_joint_positions(limb_joints)

rospy.init_node('move_left_hand')
rs = baxter_interface.RobotEnable()
rs.enable()

left = baxter_interface.Limb('left')
leftGripper = baxter_interface.Gripper('left')
pose = left.endpoint_pose()
b = True
pos = pose.popitem()
orient = pose.popitem()
prev = pos[1]

left.set_joint_position_speed(1.0)

print "-----------------------------------"
pose = left.endpoint_pose()
print "current pose: ", pose
s = raw_input("Press q to run ... ")
if s == 'q':
	moveTo(Point(0.8365484999950544,0.039402821894069295,0.11762842392850739)) # Arm Out front
	moveRel(-0.2,0,0) #Move IN
	moveRel(0,0,-0.2) #Move DOWN
	moveRel(0,-0.2,0) #Move LEFT
	moveRel(0,0,0.2) #Move UP
	moveRel(0,0.2,0) #Move RIGHT
	moveRel(0,0.2,0) #Move RIGHT