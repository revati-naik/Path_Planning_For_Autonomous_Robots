import os
import cv2
import copy
import numpy as np
import heapq
import matplotlib.pyplot as plt
import pickle

import sys
sys.dont_write_bytecode = True

import actions
import obstacles
import node
import utils
import visualization as viz
import univ



##
## Gets the A-Star path.
## In this algorithm, the heuristic cost from
## current node to goal node is not considered.
##
## :param      input_map:  The input map
## :type       input_map:  { type_description }
##
def aStar(start_pos, goal_pos, robot_radius, clearance, step_size, theta=30, duplicate_step_thresh=0.5, duplicate_orientation_thresh=30):

	start_r, start_c = start_pos
	goal_r, goal_c = goal_pos

	start_node = node.Node(current_coords=(start_r, start_c), parent_coords=None, orientation=0, parent_orientation=None, movement_cost=0, goal_cost=utils.euclideanDistance(start_pos, goal_pos))
	goal_node = node.Node(current_coords=(goal_r, goal_c), parent_coords=None, orientation=None, parent_orientation=None, movement_cost=None, goal_cost=0)


	# Saving a tuple with total cost and the state node
	minheap = [((start_node.movement_cost + start_node.goal_cost), start_node)]
	heapq.heapify(minheap)

	# defining the visited node like this avoids checking if two nodes are duplicate. because there is only 1 position to store the visited information for all the nodes that lie within this area.
	visited = {}
	visited[(utils.valRound(start_r), utils.valRound(start_c), 0)] = start_node 	# marking the start node as visited

	viz_visited_coords = [start_node]

	while len(minheap) > 0:
		_, curr_node = heapq.heappop(minheap)

		# if curr_node.isDuplicate(goal_node):
		if curr_node.goal_cost < (1.5 * step_size):
			print("Reached Goal!")
			print("Current node:---")
			curr_node.printNode()
			print("Goal node:---")
			goal_node.printNode()

			# backtrack to get the path
			path = actions.backtrack(curr_node, visited)
			# path = None 	# FOR NOW FOR DEBUGGING PURPOSES

			return (path, viz_visited_coords)

		# for row_step, col_step in movement_steps:
		for angle in range(-60, 61, theta):
			# Action Move
			# next_node = actions.actionMove(curr_node, row_step, col_step)
			next_node = actions.actionMove(current_node=curr_node, theta_step=angle, linear_step=step_size, goal_position=goal_node.current_coords)

			if next_node is not None:
				# if hit an obstacle, ignore this movement
				if obstacles.withinObstacleSpace((next_node.current_coords[1], next_node.current_coords[0]), robot_radius, clearance):
					continue

				# Check if the current node has already been visited.
				# If it has, then see if the current path is better than the previous one
				# based on the total cost = movement cost + goal cost
				node_state = (utils.valRound(next_node.current_coords[0]), utils.valRound(next_node.current_coords[1]), utils.orientationBin(next_node.orientation, theta))
				
				if node_state in visited:
					# if current cost is a better cost
					# if (next_node.movement_cost + next_node.goal_cost) < (visited[node_state].movement_cost + visited[node_state].goal_cost):
					if (next_node < visited[node_state]):
						visited[node_state].current_coords = next_node.current_coords
						visited[node_state].parent_coords = next_node.parent_coords
						visited[node_state].orientation = next_node.orientation
						visited[node_state].parent_orientation = next_node.parent_orientation
						visited[node_state].movement_cost = next_node.movement_cost
						visited[node_state].goal_cost = next_node.goal_cost

						h_idx = utils.findInHeap(next_node, minheap)
						if (h_idx > -1):
							minheap[h_idx] = ((next_node.movement_cost + next_node.goal_cost), next_node)
				else:
					# visited.append(next_node)
					visited[node_state] = next_node
					heapq.heappush(minheap, ((next_node.movement_cost + next_node.goal_cost), next_node))

					viz_visited_coords.append(next_node)

		heapq.heapify(minheap)


def testMain():
	path, viz_nodes = aStar(start_pos=(5,5), goal_pos=(50,50), robot_radius=0, clearance=0, step_size=5, theta=30, duplicate_step_thresh=0.5, duplicate_orientation_thresh=30)

	univ.function(viz_nodes, path)


if __name__ == '__main__':
	testMain()