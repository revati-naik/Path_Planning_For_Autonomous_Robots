from __future__ import division
import os
import cv2
import sys
import copy
import numpy as np
sys.dont_write_bytecode = True

import a_star


def euclideanDistance(state1, state2):
	"""
	To find the euclidean distance between two coordinates. 
	Used in the A_star algorithm.

	:param      state1:  The state 1
	:type       state1:  tuple
	:param      state2:  The state 2
	:type       state2:  tuple

	:returns:   Euclidean distance between the two states
	:rtype:     float
	"""
	return np.sqrt(((state1[0] - state2[0]) ** 2) + ((state1[1] - state2[1]) ** 2))


def findInHeap(node, node_list):
	"""
	Finds a node in heap based on only the current coordinate values.

	:param      node:       The node to be searched
	:type       node:       Node
	:param      node_list:  The search space
	:type       node_list:  list of Nodes

	:returns:   index position of where the node is found (-1 if not found)
	:rtype:     int
	"""
	node_list_coords = [item[1].current_coords for item in node_list]
	if node.current_coords in node_list_coords:
		return node_list_coords.index(node.current_coords)
	return -1


def orientationBin(angle, bin_size):
	"""
	Returns the bin for the given angle

	:param      angle:     The angle
	:type       angle:     float
	:param      bin_size:  The bin size
	:type       bin_size:  float

	:returns:   The bin to which the angle belongs in the visited dictionary
	:rtype:     float
	"""
	return (((angle % 360) // bin_size) * bin_size)


def getKey(x, y, theta):
	"""
	Gets the key for the given tuple

	:param      x:      X - coordinate
	:type       x:      number
	:param      y:      Y - coordinate
	:type       y:      number
	:param      theta:  The orientation
	:type       theta:  number

	:returns:   The key.
	:rtype:     tuple
	"""
	x_new = int(np.floor(x*10))/10
	y_new = int(np.floor(y*10))/10
	t_bin = orientationBin(theta, a_star.THETA_BIN_SIZE)

	return ((x_new, y_new, t_bin))



def testMain():
	pass


if __name__ == '__main__':
	testMain()