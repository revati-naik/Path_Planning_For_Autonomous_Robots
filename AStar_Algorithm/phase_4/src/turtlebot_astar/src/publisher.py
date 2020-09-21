#!/usr/bin/python
from __future__ import print_function
import os
import sys
import rospy
import numpy as np
import math
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Point
import rospkg
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
sys.dont_write_bytecode = True

import utils


# Initialiasing global variables for the current location of TurtleBot
x = 0.0
y = 0.0
theta = 0.0

# Variable to store the next location from the path list
goal = Point()
goal.x = x
goal.y = y

path_itr = 1


def shutdownProcedure():
	print("turtlebot_move Node shutting down! Manually initiated shutdown once the goal node is reached by the turtlebot.")


def newOdom(msg, args):
	"""
	Callback Function for Subscriber to get Robot's Current Location

	:param      msg:   Robots Odometry
	:type       msg:   nav_msgs/Odometry
	:param      args:  (path_list, cmd_vel publisher)
	:type       args:  tuple
	"""
	global x
	global y
	global theta

	global goal
	global path_itr

	path_list = args[0]
	cmd_vel_pub = args[1]

	x = msg.pose.pose.position.x
	y = msg.pose.pose.position.y

	rot_q = msg.pose.pose.orientation
	roll, pitch, theta = euler_from_quaternion((rot_q.x, rot_q.y, rot_q.z, rot_q.w))

	if utils.euclideanDistance((goal.x, goal.y), (x, y)) < 0.2:
		goal.x, goal.y = path_list[path_itr].getXYCoords()
		path_itr += 1

		if path_itr >= len(path_list):
			speed = Twist()
			speed.linear.x = 0.0
			speed.angular.z = 0.0
			cmd_vel_pub.publish(speed)
			rospy.signal_shutdown("Reached the last node of the path_list!!! Yayyy!")


def main():
	"""
	Main funtion that initialises the ROS node, creates the publisher and the subscriber for the TurtleBot
	"""

	# Initialising ROS node
	rospy.init_node("turtlebot_move")

	# Reading parameters from the launch file
	npy_path = rospy.get_param("/publish_velocity/npy_file_path")

	# Reading the generated A* path from the .npy file
	# rospack = rospkg.RosPack()
	# npy_path = os.path.join(rospack.get_path('turtlebot_astar'), 'src/path_dumps/path_final.npy')
	robot_path_list = np.load(npy_path, allow_pickle=True)

	global goal
	goal.x, goal.y = robot_path_list[0].getXYCoords()

	# Creating the Publisher and the Subscriber
	pub = rospy.Publisher("/cmd_vel", Twist, queue_size=1)
	sub = rospy.Subscriber("/odom", Odometry, newOdom, (robot_path_list, pub))

	r = rospy.Rate(4)
	speed = Twist()

	try:
		while not rospy.is_shutdown():

			inc_x = goal.x - x
			inc_y = goal.y - y

			angle_to_goal = math.atan2(inc_y, inc_x)

			if abs(angle_to_goal - theta) < 0.1:
				speed.linear.x = 0.5
				speed.angular.z = 0.0
			elif (angle_to_goal - theta) < 0:
				speed.linear.x = 0.0
				speed.angular.z = -0.3
			else:
				speed.linear.x = 0.0
				speed.angular.z = 0.3

			# Publishing the Velocity Inputs for the TurtleBot on the topic /cmd_vel
			pub.publish(speed)
			r.sleep()

	except rospy.exceptions.ROSInterruptException as ros_int:
		print(ros_int)
	except Exception as e:
		raise e


if __name__ == '__main__':
	main()
