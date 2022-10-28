#! /usr/bin/env python

import rospy
import tf
import time

def getActualState(pose):
	actualX = pose[0]
	actualY = pose[1]
	if((-3.3 < actualX < -1.5) and (-6.3 < actualY < -3.9)):
		print("State 0")
		return 0
	elif((-1.5 < actualX < 0.3) and (-6 < actualY < -3.9)):
		print("State 1")
		return 1
	elif((0.3 < actualX < 3.6) and (-5.7 < actualY < -3.9)):
		print("State 2")
		return 2
	elif((0.3 < actualX < 3.6) and (-3.9 < actualY < -2.1)):
		print("State 3")
		return 3
	elif((-3.6 < actualX < 0.3) and (-3.9 < actualY < -2.1)):
		print("State 4")
		return 4
	elif((-3.9 < actualX < -1.5) and (-2.1 < actualY < -0.3)):
		print("State 5")
		return 5
	elif((-1.5 < actualX < 0.3) and (-2.1 < actualY < -0.3)):
		print("State 6")
		return 6
	elif((0.3 < actualX < 3.6) and (-2.1 < actualY < -0.3)):
		print("State 7")
		return 7
	elif((0.3 < actualX < 3.6) and (-0.3 < actualY < 2.4)):
		print("State 8")
		return 8
	elif((-2.7 < actualX < 0.3) and (-0.3 < actualY < 1.8)):
		print("State 9")
		return 9
	else:
		print("TRANSITION")
		return

rospy.init_node('newNode')
listener = tf.TransformListener()
listener.waitForTransform("/map", "/base_link", rospy.Time(0), rospy.Duration(5.0))
while not rospy.is_shutdown():
	try:
		(trans,rot) = listener.lookupTransform("/map", "/base_link", rospy.Time(0))
		print(trans)
		getActualState(trans)
		time.sleep(5)
		listener.waitForTransform("/map", "/base_link", rospy.Time(0), rospy.Duration(5.0))
	except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
		continue
