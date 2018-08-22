#!/usr/bin/env python


import rospy
import math
import tf
from geometry_msgs.msg import Pose2D
from std_msgs.msg import Int16

pose = Pose2D()
yaw = 0

#Takes yaw and pose, combines and publishes as a transfer frame
def publishTF():
	global yaw, pose
	br = tf.TransformBroadcaster()
	br.sendTransform((pose.x, pose.y, 0), tf.transformations.quaternion_from_euler(0, 0, yaw), rospy.Time.now(), "groundRobot", "map")

#Updates pose with new pose every time it is received by the subscriber
def poseCallback(pose_n):
	global pose
	pose = pose_n
	publishTF()
	
#Updates yaw value every time a new one is received by the subscriber
def yawCallback(yaw_n):
	global yaw
	#transfer frame uses radians, so yaw must be converted
	yaw = math.radians(yaw_n.data)

#create the node with a subscriber for yaw and a subscriber for pose
if __name__ == '__main__':
	rospy.init_node('groundRobotTFPublisher')
	rospy.Subscriber("/groundRobot/Pose", Pose2D, poseCallback)
	rospy.Subscriber("/groundRobot/Yaw", Int16, yawCallback)	
	rospy.spin()
