import sys
sys.dont_write_bytecode = True

import utils

class Node(object):
	
	##
	## Constructs a new instance.
	##
	## :param      current_coords:  The current coordinates
	## :type       current_coords:  (row, col)
	## :param      parent_coords:   The parent coordinates
	## :type       parent_coords:   (row, col)
	## :param      movement_cost:   Movement cost from start node to current node
	## :type       movement_cost:   float
	## :param      goal_cost:       The goal cost
	## :type       goal_cost:       float
	##
	def __init__(self, 
				 current_coords, 
				 parent_coords, 
				 orientation,
				 parent_orientation,
				 movement_cost, 
				 goal_cost):
		self.current_coords = current_coords
		self.parent_coords = parent_coords
		self.orientation = orientation % 360 if orientation is not None else None
		self.parent_orientation = parent_orientation % 360 if parent_orientation is not None else None
		self.movement_cost = movement_cost
		self.goal_cost = goal_cost


	def __gt__(self, other):
		return ((self.movement_cost + self.goal_cost) > (other.movement_cost + other.goal_cost))


	def __lt__(self, other):
		return ((self.movement_cost + self.goal_cost) < (other.movement_cost + other.goal_cost))


	def __ge__(self, other):
		return ((self.movement_cost + self.goal_cost) >= (other.movement_cost + other.goal_cost))


	def __le__(self, other):
		return ((self.movement_cost + self.goal_cost) <= (other.movement_cost + other.goal_cost))


	def __eq__(self, other):
		return ((self.movement_cost + self.goal_cost) == (other.movement_cost + other.goal_cost))


	def getXYCoords(self):
		if self.current_coords is not None:
			return (self.current_coords[1], self.current_coords[0])
		return None


	def getRowColCoords(self):
		if self.current_coords is not None:
			return (self.current_coords)
		return None

	
	def getParentXYCoords(self):
		if self.parent_coords is not None:
			return (self.parent_coords[1], self.parent_coords[0])
		return None


	##
	## Determines whether the specified random node is a duplicate of the current node.
	##
	## :param      random_node:  The random node
	## :type       random_node:  Node
	##
	## :returns:   True if the specified random node is duplicate, False otherwise.
	## :rtype:     boolean
	##
	def isDuplicate(self, random_node):
		if random_node.orientation is None:
			return (utils.euclideanDistance(self.current_coords, random_node.current_coords) < 0.5)
		
		return ((utils.euclideanDistance(self.current_coords, random_node.current_coords) < 0.5) and (abs(self.orientation - random_node.orientation) < 30))


	##
	## Match all attributes of the current object with the random_node
	##
	## :param      random_node:  The node to match the current object with
	## :type       random_node:  Node
	##
	## :returns:   True if all the attributes match, False otherwise
	## :rtype:     boolean
	##
	def deepMatch(self, random_node):
		return (self.current_coords == random_node.current_coords and
				self.parent_coords == random_node.parent_coords and
				self.orientation == random_node.orientation and
				self.movement_cost == random_node.movement_cost and
				self.goal_cost == random_node.goal_cost)

	
	##
	## Match only the current_coords attributes of the current object with the random_node
	##
	## :param      random_node:  The node to match the current object with
	## :type       random_node:  Node
	##
	## :returns:   True if current_coords attribute matches, False otherwise
	## :rtype:     boolean
	##
	def shallowMatch(self, random_node):
		return (self.current_coords == random_node.current_coords)


	##
	## Prints all the information regarding the current configuration.
	##
	def printNode(self):
		print "current_coords:\t", self.current_coords
		print "parent_coords:\t", self.parent_coords
		print "orientation:\t", self.orientation
		print "movement_cost:\t", self.movement_cost
		print "goal_cost:\t", self.goal_cost
