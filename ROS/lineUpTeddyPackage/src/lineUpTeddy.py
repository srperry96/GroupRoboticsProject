#!/usr/bin/env python
#ALL OF THIS IS UNTESTED
import rospy
from geometry_msgs.msg import Point
from geometry_msgs.msg import Twist

cameraResX = 1024 #resolution of camera image X (width)
k = 0.5

def teddyPosCallback(teddyPosition):
	global cameraResX
	global k
	pub = rospy.Publisher("/groundRobot/MovementControl", Twist, queue_size=10)
	error = teddyPosition.x / cameraResX
	ctrl = k * error
	msg = Twist()
	msg.angular.z = ctrl
	msg.angular.x = 0; msg.angular.y = 0
	msg.linear.x = 0; msg.linear.y = 0; msg.linear.z = 0
	pub.publish(msg)

if __name__ == '__main__':
	rospy.init_node('groundLineUpTeddy')
	rospy.Subscriber("/greenprint", Point, teddyPosCallback)
	rospy.spin()
