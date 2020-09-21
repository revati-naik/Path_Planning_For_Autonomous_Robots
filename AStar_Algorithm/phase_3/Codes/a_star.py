from __future__ import print_function, division
import os
import sys
import time
import heapq
import numpy as np
import matplotlib.pyplot as plt
sys.dont_write_bytecode = True

import actions_new as an
import obstacles
import node
import utils
import visualization as viz
# import univ


THETA_BIN_SIZE = 30
GOAL_REACH_THRESH = 0.2 	# units in meters

# Robot radius for TurtleBot3
ROBOT_RADIUS = 0.105

# Output directory
OUTPUT_DIR = "./output"


def a_star(start_rc, goal_rc, orientation, rpm1=10, rpm2=20, clearance=0.2, viz_please=False):
	"""
	A-star algorithm given the start and the goal nodes.

	:param      start_rc:     The start position
	:type       start_rc:     tuple
	:param      goal_rc:      The goal position
	:type       goal_rc:      tuple
	:param      orientation:  The orientation
	:type       orientation:  number
	:param      rpm1:         The rpm 1
	:type       rpm1:         number
	:param      rpm2:         The rpm 2
	:type       rpm2:         number
	:param      clearance:    The clearance
	:type       clearance:    number
	:param      viz_please:   Whether to visualize or not
	:type       viz_please:   boolean

	:returns:   Returns path nodes and the list of visited nodes.
	:rtype:     tuple
	"""

	"""
	Inputs
	"""
	start_node = node.Node(current_coords=start_rc, parent_coords=None, orientation=0, parent_orientation=None, action=None, movement_cost=0, goal_cost=utils.euclideanDistance(start_rc, goal_rc))
	print("-----------------------")
	print("Start Node:")
	start_node.printNode()
	print("-----------------------")
	goal_node = node.Node(current_coords=goal_rc, parent_coords=None, orientation=None, parent_orientation=None, action=None, movement_cost=None, goal_cost=0)

	"""
	Initializations
	"""
	action_set = [(0, rpm1),
				  (rpm1, 0),
				  (0, rpm2),
				  (rpm2, 0),
				  (rpm1, rpm2),
				  (rpm2, rpm1),
				  (rpm1, rpm1),
				  (rpm2, rpm2)]

	min_heap = [((start_node.movement_cost + start_node.goal_cost), start_node)]
	heapq.heapify(min_heap)

	visited = {}
	visited.update({(utils.getKey(start_rc[0], start_rc[1], start_node.orientation)): start_node})

	visited_viz_nodes = [start_node]

	"""
	Initialize the plot figures if the visualization flag is true.
	Also mark the start and the goal nodes.
	"""
	if viz_please:
		fig, ax = plt.subplots()
		ax.set(xlim=(an.MIN_COORDS[0], an.MAX_COORDS[0]), ylim=(an.MIN_COORDS[1], an.MAX_COORDS[1]))
		# ax.set(xlim=(-5, 5), ylim=(-5, 5))
		ax.set_aspect('equal')

		obstacles.generateMap(plotter=ax)

		viz.markNode(start_node, plotter=ax, color='#00FF00', marker='o')
		viz.markNode(goal_node, plotter=ax, color='#FF0000', marker='^')

		plt.ion()

	"""
	Run the loop for A-star algorithm till the min_heap queue contains no nodes.
	"""
	while (len(min_heap) > 0):
		_, curr_node = heapq.heappop(min_heap)

		# Consider all the action moves for all the selected nodes.
		for action in action_set:
			new_node = an.actionMove(current_node=curr_node, next_action=action, goal_position=goal_rc, clearance=clearance)

			# Check if all the nodes are valid or not.
			if (new_node is not None):

				"""
				Check if the current node is a goal node.
				"""
				if new_node.goal_cost < GOAL_REACH_THRESH:
					print("Reached Goal!")
					print("Final Node:")
					new_node.printNode()
					visited_viz_nodes.append(new_node)

					path = an.backtrack(new_node, visited)
					print("------------------------")
					if viz_please:
						viz.plot_curve(new_node, plotter=ax, color="red")
						viz.plotPath(path, rev=True, pause_time=0.5, plotter=ax, color="lime", linewidth=4)

						plt.ioff()
						plt.show()
					return (path, visited_viz_nodes)

				"""
				Mark node as visited,
				Append to min_heap queue,
				Update if already visited.
				"""
				node_key = (utils.getKey(new_node.current_coords[0], new_node.current_coords[1], new_node.orientation))

				if node_key in visited:
					if new_node < visited[node_key]:
						visited[node_key] = new_node
						min_heap.append(((new_node.movement_cost + new_node.goal_cost), new_node))

				else:
					if viz_please:
						viz.plot_curve(new_node, plotter=ax, color="red")
						plt.show()
						plt.pause(0.001)

					visited.update({node_key: new_node})
					min_heap.append(((new_node.movement_cost + new_node.goal_cost), new_node))

					visited_viz_nodes.append(new_node)

		# Heapify the min heap to update the minimum node in the list.
		heapq.heapify(min_heap)


def main():
	"""
	Main function only to test the code.
	"""
	start_rc = (-4, -4)
	goal_rc = (4, 4)
	theta = 0

	rpm1 = 10
	rpm2 = 20

	clearance = 0.2

	start_time = time.clock()
	path, visited_viz_nodes = a_star(start_rc=start_rc, goal_rc=goal_rc, orientation=theta, rpm1=rpm1, rpm2=rpm2, clearance=clearance, viz_please=False)
	print("Time taken for Astar:", time.clock() - start_time)

	np.save("./path_dumps/path.npy", path)
	np.save("./path_dumps/visited_viz_nodes.npy", visited_viz_nodes)

	print("Number of visited nodes:", len(visited_viz_nodes))
	print("Number of nodes in path:", len(path))

	plotter = viz.initPlot(start_rc[::-1], goal_rc[::-1], title="Final Plotting")
	plt.savefig(os.path.join(OUTPUT_DIR, "1.png"))
	plt.ion()
	i = 2
	i = viz.plotPath(path=visited_viz_nodes, rev=False, pause_time=0.001, plotter=plotter, color="blue", linewidth=1, write_path_prefix=i, show=False, skip_frames=25)
	i = viz.plotPath(path=path, rev=True, pause_time=0.001, plotter=plotter, color="lime", linewidth=3, write_path_prefix=i, show=False, skip_frames=1)
	plt.ioff()
	print("Done with plots.")
	plt.show()


if __name__ == '__main__':
	main()
