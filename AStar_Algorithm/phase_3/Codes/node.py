import sys
sys.dont_write_bytecode = True

import utils

class Node(object):
	
	##
	## Constructs a new instance.
	##
	## :param      current_coords:      The current coordinates
	## :type       current_coords:      (row, col)
	## :param      parent_coords:       The parent coordinates
	## :type       parent_coords:       (row, col)
	## :param      orientation:         The orientation
	## :type       orientation:         float
	## :param      parent_orientation:  The parent orientation
	## :type       parent_orientation:  float
	## :param      action:              The action
	## :type       action:              tuple of rpms for both wheels
	## :param      movement_cost:       Movement cost from start node to current
	##                                  node
	## :type       movement_cost:       float
	## :param      goal_cost:           The goal costfrom current to goal node
	## :type       goal_cost:           float
	##
	def __init__(self, 
				 current_coords, 
				 parent_coords=None, 
				 orientation=0,
				 parent_orientation=None,
				 action=None,
				 movement_cost=None, 
				 goal_cost=None,):
		self.current_coords = current_coords
		self.parent_coords = parent_coords
		self.orientation = orientation % 360 if orientation is not None else None
		self.parent_orientation = parent_orientation % 360 if parent_orientation is not None else None
		self.action = action
		self.movement_cost = movement_cost
		self.goal_cost = goal_cost


	def __gt__(self, other):
		"""
		Greater-than comparison operator to compare based on the total costs.
	
		:param      other:  The other node
		:type       other:  Node
	
		:returns:   The result of the greater-than comparison
		:rtype:     boolean
		"""
		return ((self.movement_cost + self.goal_cost) > (other.movement_cost + other.goal_cost))


	def __lt__(self, other):
		"""
		Less-than comparison operator to compare based on the total costs.
	
		:param      other:  The other node
		:type       other:  Node
	
		:returns:   The result of the less-than comparison
		:rtype:     boolean
		"""
		return ((self.movement_cost + self.goal_cost) < (other.movement_cost + other.goal_cost))


	def __ge__(self, other):
		"""
		Greater-than-or-equal comparison operator to compare based on the total costs.
	
		:param      other:  The other node
		:type       other:  Node
	
		:returns:   The result of the greater-than-or-equal comparison
		:rtype:     boolean
		"""
		return ((self.movement_cost + self.goal_cost) >= (other.movement_cost + other.goal_cost))


	def __le__(self, other):
		"""
		Less-than-or-equal comparison operator to compare based on the total costs.
	
		:param      other:  The other node
		:type       other:  Node
	
		:returns:   The result of the less-than-or-equal comparison
		:rtype:     boolean
		"""
		return ((self.movement_cost + self.goal_cost) <= (other.movement_cost + other.goal_cost))


	def __eq__(self, other):
		"""
		Equality operator to compare based on the total costs.
	
		:param      other:  The other node
		:type       other:  Node
	
		:returns:   The result of the equality
		:rtype:     boolean
		"""
		return ((self.movement_cost + self.goal_cost) == (other.movement_cost + other.goal_cost))


	def getXYCoords(self):
		"""
		Gets the xy coordinates.
	
		:returns:   The xy coordinates.
		:rtype:     tuple
		"""
		if self.current_coords is not None:
			return (self.current_coords[1], self.current_coords[0])
		return None


	def getRowColCoords(self):
		"""
		Gets the row col coordinates.
	
		:returns:   The row col coordinates.
		:rtype:     tuple
		"""
		if self.current_coords is not None:
			return (self.current_coords)
		return None

	
	def getParentXYCoords(self):
		"""
		Gets the parent xy coordinates.
	
		:returns:   The parent xy coordinates.
		:rtype:     tuple
		"""
		if self.parent_coords is not None:
			return (self.parent_coords[1], self.parent_coords[0])
		return None


	def isDuplicate(self, random_node, dist_thresh, orient_thresh):
		"""
		Determines whether the specified random node is a duplicate of the current node.
	
		:param      random_node:    The node to match the current object with
		:type       random_node:    Node
		:param      dist_thresh:    The distance threshold for duplicacy
		:type       dist_thresh:    float
		:param      orient_thresh:  The orientation threshold for duplicacy
		:type       orient_thresh:  float
	
		:returns:   True if duplicate, False otherwise.
		:rtype:     boolean
		"""
		if random_node.orientation is None:
			return (utils.euclideanDistance(self.current_coords, random_node.current_coords) < dist_thres)
		
		return ((utils.euclideanDistance(self.current_coords, random_node.current_coords) < dist_thresh) and (abs(self.orientation - random_node.orientation) < orient_thresh))


	def deepMatch(self, random_node):
		"""
		Match all attributes of the current object with the random_node
	
		:param      random_node:  
		:type       random_node:  
	
		:returns:   True if all the attributes match, False otherwise
		:rtype:     boolean
		"""
		return (self.current_coords == random_node.current_coords and
				self.parent_coords == random_node.parent_coords and
				self.orientation == random_node.orientation and
				self.movement_cost == random_node.movement_cost and
				self.goal_cost == random_node.goal_cost)

	
	def shallowMatch(self, random_node):
		"""
		Match only the current_coords attributes of the current object with the random_node
	
		:param      random_node:  The node to match the current object with
		:type       random_node:  Node
	
		:returns:   True if current_coords attribute matches, False otherwise
		:rtype:     boolean
		"""
		return (self.current_coords == random_node.current_coords)


	def printNode(self):
		"""
		Prints all the information regarding the current configuration.
		"""
		print "current_coords:\t", self.current_coords
		print "parent_coords:\t", self.parent_coords
		print "orientation:\t", self.orientation
		print "Action:	\t", self.action
		print "movement_cost:\t", self.movement_cost
		print "goal_cost:\t", self.goal_cost
