#! /usr/bin/env python
##Script that given the two policies calculates the path and sends it to
##the robot in form of goal points using the navigation stack

import rospy
import tf
import numpy as np
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal

S = [0,1,2,3,4,5,6,7,8,9]
A = [0,1,2,3,4]

#UP
P0 = np.array([[0.2, 0.8, 0, 0, 0, 0, 0, 0, 0, 0],
    [0, 0.2, 0.8, 0, 0, 0, 0, 0, 0, 0],
    [0, 0, 1, 0, 0, 0, 0, 0, 0, 0],
    [0, 0, 0, 1, 0, 0, 0, 0, 0, 0],
    [0, 0, 0, 0.8, 0.2, 0, 0, 0, 0, 0],
    [0, 0, 0, 0, 0, 0.2, 0.8, 0, 0, 0],
    [0 ,0 ,0, 0, 0, 0, 0.2, 0.8, 0, 0],
    [0, 0, 0, 0, 0, 0, 0, 1, 0, 0],
    [0, 0, 0, 0, 0, 0, 0, 0, 1, 0],
    [0, 0, 0, 0, 0, 0, 0, 0, 0.8, 0.2]])

#DOWN
P1 = np.array([[1, 0, 0, 0, 0, 0, 0, 0, 0, 0],
    [0.8, 0.2, 0, 0, 0, 0, 0, 0, 0, 0],
    [0, 0.8, 0.2, 0, 0, 0, 0, 0, 0, 0],
    [0, 0, 0, 0.2, 0.8, 0, 0, 0, 0, 0],
    [0, 0, 0, 0, 1, 0, 0, 0, 0, 0],
    [0, 0, 0, 0, 0, 1, 0, 0, 0, 0],
    [0, 0, 0, 0, 0, 0.8, 0.2, 0, 0, 0],
    [0, 0, 0, 0, 0, 0, 0.6, 0.4, 0, 0],
    [0, 0, 0, 0, 0, 0, 0, 0, 0.2, 0.8],
    [0, 0, 0, 0, 0, 0, 0, 0, 0, 1]])

#LEFT
P2 = np.array([[0.2, 0, 0, 0, 0.8, 0, 0, 0, 0, 0],
    [0, 0.2, 0, 0, 0.8, 0, 0, 0, 0, 0],
    [0, 0, 0.2, 0.7, 0.1, 0, 0, 0, 0, 0],
    [0, 0, 0, 0.2, 0, 0, 0.1, 0.7, 0, 0],
    [0, 0, 0, 0, 0.2, 0.4, 0.4, 0, 0, 0],
    [0, 0, 0, 0, 0, 1, 0, 0, 0, 0],
    [0, 0, 0, 0, 0, 0, 1, 0, 0, 0],
    [0, 0, 0, 0, 0, 0, 0, 0.2, 0.8, 0],
    [0, 0, 0, 0, 0, 0, 0, 0, 1, 0],
    [0, 0, 0, 0, 0, 0, 0, 0, 0, 1]])

#RIGHT
P3 = np.array([[1, 0, 0, 0, 0, 0, 0, 0, 0, 0],
    [0, 1, 0, 0, 0, 0, 0, 0, 0, 0],
    [0, 0, 1, 0, 0, 0, 0, 0, 0, 0],
    [0, 0, 0.8, 0.2, 0, 0, 0, 0, 0, 0],
    [0.35, 0.35, 0.1, 0, 0.2, 0, 0, 0, 0, 0],
    [0, 0, 0, 0, 0.8, 0.2, 0, 0, 0, 0],
    [0, 0, 0, 0.1, 0.7, 0, 0.2, 0, 0, 0],
    [0, 0, 0, 0.8, 0, 0, 0, 0.2, 0, 0],
    [0, 0, 0, 0, 0, 0, 0, 0.8, 0.2, 0],
    [0, 0, 0, 0, 0, 0, 0, 0, 0, 1]])

#STAY
P4 = np.array([[1, 0, 0, 0, 0, 0, 0, 0, 0, 0],
    [0, 1, 0, 0, 0, 0, 0, 0, 0, 0],
    [0, 0, 1, 0, 0, 0, 0, 0, 0, 0],
    [0, 0, 0, 1, 0, 0, 0, 0, 0, 0],
    [0, 0, 0, 0, 1, 0, 0, 0, 0, 0],
    [0, 0, 0, 0, 0, 1, 0, 0, 0, 0],
    [0, 0, 0, 0, 0, 0, 1, 0, 0, 0],
    [0, 0, 0, 0, 0, 0, 0, 1, 0, 0],
    [0, 0, 0, 0, 0, 0, 0, 0, 1, 0],
    [0, 0, 0, 0, 0, 0, 0, 0, 0, 1]])

P = np.array([P0, P1, P2, P3, P4])

def changeToMeanValues(path):
	pathMean=[]
	for node in path:
		if(node == 0):
			pathMean+=[[-2.4,-4.6]]
		elif(node == 1):
			pathMean+=[[-0.6,-4.4]]
		elif(node == 4):
			pathMean+=[[-1.65,-3]]
		elif(node == 2):
			pathMean+=[[1.95,-4.4]]
		elif(node == 3):
			pathMean+=[[1.95,-3]]
		elif(node == 5):
			pathMean+=[[-2.7,-1]]
		elif(node == 6):
			pathMean+=[[-0.6,-1.2]]
		elif(node == 7):
			pathMean+=[[1.8,-1.2]]
		elif(node == 8):
			pathMean+=[[1.95,0.7]]
		elif(node == 9):
			pathMean+=[[-1.2,0.7]]
	return pathMean

def movebase_client(path):

    client = actionlib.SimpleActionClient('move_base',MoveBaseAction)
    client.wait_for_server()

    for node in path:
	    goal = MoveBaseGoal()
	    goal.target_pose.header.frame_id = "map"
	    goal.target_pose.header.stamp = rospy.Time.now()
	    goal.target_pose.pose.position.x = node[0]
	    goal.target_pose.pose.position.y = node[1]
	    #arbitrary
	    goal.target_pose.pose.orientation.w = 1.57

	    client.send_goal(goal)
	    wait = client.wait_for_result()

    if not wait:
        rospy.logerr("Action server not available!")
        rospy.signal_shutdown("Action server not available!")
    else:
        return client.get_result()

def calculatepath(P, S, A,initialState, policy, goal_state):
    state = initialState
    path=[]
    n_states, n_actions = len(S), len(A)
    while(state != goal_state):
        action = policy[state]
        state = np.random.choice(n_states, p=P[action][state])
        path+=[state]
    return path

def getActualState(pose):
	actualX = pose[0]
	actualY = pose[1]
	if((-3.3 < actualX < -1.5) and (-6 < actualY < -3.9)):
		print("State 0")
		return 0
	elif((-1.5 < actualX < 0.3) and (-5.7 < actualY < -3.9)):
		print("State 1")
		return 1
	elif((0.3 < actualX < 3.6) and (-5.4 < actualY < -3.9)):
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

policy1 = (4, 1, 1, 3, 3, 0, 0, 3, 3, 0)
policy2 = (0, 0, 2, 2, 0, 0, 0, 2, 1, 4)



rospy.init_node('turtle_tf_listener')
listener = tf.TransformListener()

#GET CURRENT STATE
listener.waitForTransform("/map", "/base_link", rospy.Time(0), rospy.Duration(3.0))
(trans,rot) = listener.lookupTransform("/map", "/base_link", rospy.Time(0))
initial_state = getActualState(trans)

#CALCULATE PATH
path1 = calculatepath(P, S, A, initial_state, policy1, 0)
print(path1)
path1 = changeToMeanValues(path1)
if(path1 == []):
	print("Failed to get path!")
	exit()

#MAKE THE ROBOT DO THE PATH
try:
    result = movebase_client(path1)
    if result:
        rospy.loginfo("Goal execution done!")
except rospy.ROSInterruptException:
    rospy.loginfo("Navigation test finished.")


#GET CURRENT STATE
listener.waitForTransform("/map", "/base_link", rospy.Time(0), rospy.Duration(3.0))
(trans,rot) = listener.lookupTransform("/map", "/base_link", rospy.Time(0))
initial_state = getActualState(trans)
print("State Calculated")
#CALCULATE PATH
path2 = calculatepath(P, S, A, initial_state, policy2, 9)
print(path2)
path2 = changeToMeanValues(path2)

#MAKE THE ROBOT DO THE PATH
try:
    result = movebase_client(path2)
    if result:
        rospy.loginfo("Goal execution done!")
except rospy.ROSInterruptException:
    rospy.loginfo("Navigation test finished.")
