import os
import cv2
import copy
import numpy as np
import heapq
import time

import sys
sys.dont_write_bytecode = True

import actions
import obstacles
import node
import utils
import a_star
import univ


def main():


	# User input for the start state
	start_c, start_r, start_orient = map(float, raw_input("Enter starting coordinates (x y) and orientation: ").split())
	# User input for goal state
	goal_c, goal_r = map(float, raw_input("Enter goal coordinates (x y): ").split())
	
	radius = float(input("Enter the robot radius: "))
	clearance = float(input("Enter the clearance: "))
	step_size = float(input("Enter the robot step_size: "))

	# check if the start node lies withing the map and not on obstacles
	if (start_r < actions.MIN_COORDS[1]) or (start_r >= actions.MAX_COORDS[1]) or (start_c < actions.MIN_COORDS[0]) or (start_c >= actions.MAX_COORDS[0]) or obstacles.withinObstacleSpace((start_c, start_r), radius, clearance):
		print("ERROR: Invalid start node. It either lies outside the map boundary or within the obstacle region.")
		sys.exit(0)

	# check if the goal node lies withing the map and not on obstacles
	if (goal_r < actions.MIN_COORDS[1]) or (goal_r >= actions.MAX_COORDS[1]) or (goal_c < actions.MIN_COORDS[0]) or (goal_c >= actions.MAX_COORDS[0]) or obstacles.withinObstacleSpace((goal_c, goal_r), radius, clearance):
		print("ERROR: Invalid goal node. It either lies outside the map boundary or within the obstacle region.")
		sys.exit(0)

	# check is step size lies between 0 and 10
	if step_size < 1 or step_size > 10:
		print("ERROR: Invalid step_size. It must lie within 1 and 10.")
		sys.exit(0)



	# write code to find the actual path using a star
	start_time = time.clock()
	path, viz_nodes = a_star.aStar(start_pos=(start_r,start_c), goal_pos=(goal_r,goal_c), robot_radius=radius, clearance=clearance, step_size=step_size, theta=30, duplicate_step_thresh=0.5, duplicate_orientation_thresh=30)
	print "Time to run A*:", time.clock() - start_time, "seconds"

	# visualize path
        goal_node = node.Node(current_coords=(goal_r, goal_c), parent_coords=None, orientation=None, parent_orientation=None, movement_cost=None, goal_cost=0)
	univ.function(viz_nodes, path, goal_node, step_size)


if __name__ == '__main__':
	main()
