#!/usr/bin/python
from __future__ import print_function
import os
import sys
import math
import numpy as np
sys.dont_write_bytecode = True

import rospy
import rospkg
from gazebo_msgs.msg import ModelState 
from geometry_msgs.msg import Twist, Point, Quaternion, Pose
from nav_msgs.msg import Odometry
from gazebo_msgs.srv import SetModelState, SpawnModel
from tf.transformations import euler_from_quaternion, quaternion_from_euler

import utils
import a_star
import obstacles
import visualization as viz
import Astar_rigid as a_star_main


# Initialiasing global variables for the current location of TurtleBot
x = 0.0
y = 0.0
theta = 0.0

# Variable to store the next location from the path list
intermediate_goal = Point()
intermediate_goal.x = x
intermediate_goal.y = y

path_itr = 1


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

	global intermediate_goal
	global path_itr

	path_list = args[0]
	cmd_vel_pub = args[1]

	x = msg.pose.pose.position.x
	y = msg.pose.pose.position.y

	rot_q = msg.pose.pose.orientation
	roll, pitch, theta = euler_from_quaternion((rot_q.x, rot_q.y, rot_q.z, rot_q.w))

	if utils.euclideanDistance((intermediate_goal.x, intermediate_goal.y), (x, y)) < 0.2:
		intermediate_goal.x, intermediate_goal.y = path_list[path_itr].getXYCoords()
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

	"""
	Reading parameters from the launch file
	"""
	start_x = rospy.get_param("start_x")
	start_y = rospy.get_param("start_y")
	start_theta = rospy.get_param("start_theta")
	goal_x = rospy.get_param("goal_x")
	goal_y = rospy.get_param("goal_y")
	rpm_1 = rospy.get_param("rpm_1")
	rpm_2 = rospy.get_param("rpm_2")
	clearance = rospy.get_param("clearance")

	"""
	Compute the shortest path between the given start and end nodes.
	"""
	optimal_path = a_star_main.findShortestPath(start_x=start_x, start_y=start_y, start_theta=start_theta, goal_x=goal_x, goal_y=goal_y, rpm1=rpm_1, rpm2=rpm_2, clearance=clearance)

	"""
	Spawn the robot in the Gazebo environment.
	"""
	# rospy.wait_for_service("gazebo/spawn_urdf_model")
	# print("Got the 'spawn_urdf_model' service!!")
	# item_name = "turtlebot3_burger"
	
	# rospack = rospkg.RosPack()
	# urdf_descrition_path = os.path.join(rospack.get_path('turtlebot3_description'), 'urdf/' + item_name + '.urdf.xacro')
	# f = open(urdf_descrition_path, "r")
	# description_xml = f.read()
	
	# q = quaternion_from_euler(0, 0, start_theta)
	# item_pose = Pose(Point(x=start_x, y=start_y, z=0), Quaternion(x=q[0], y=q[1], z=q[2], w=q[3]))
	
	# spawn_model = rospy.ServiceProxy('/gazebo/spawn_urdf_model', SpawnModel)
	# resp = spawn_model(item_name, description_xml, "/", item_pose, "world")
	# print(resp)

	"""
	Move the robot as per the optimal path generated.
	"""
	global intermediate_goal
	intermediate_goal.x, intermediate_goal.y = optimal_path[1].getXYCoords()

	# Creating the Publisher and the Subscriber
	pub = rospy.Publisher("/cmd_vel", Twist, queue_size=1)
	sub = rospy.Subscriber("/odom", Odometry, newOdom, (optimal_path, pub))

	r = rospy.Rate(4)
	speed = Twist()

	try:
		while not rospy.is_shutdown():

			inc_x = intermediate_goal.x - x
			inc_y = intermediate_goal.y - y

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
