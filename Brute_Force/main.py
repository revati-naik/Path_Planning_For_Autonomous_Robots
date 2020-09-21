import copy
import numpy as np 


#
# Check if the matrtix is solvable by checking the number of inversions
# Odd number of inversions = Not Solvable
# Even Number of inversions = SOlvable
#
# :type       inputData:  2D array (list)
# :param      inputData:  Input Data from User
#
# :returns:   True if the specified input data is solvable, False otherwise.
# :rtype:     boolean
#
def isSolvable(inputData):
		inversionCount = 0
		# Converting 2D array into 1D array
		inputDataModified = [c for r in inputData for c in r]

		# Counting the number of inversions
		for i in range(len(inputDataModified)):
			for j in range(i+1, len(inputDataModified)):
				if inputDataModified[i] != 0 and inputDataModified[j] !=0:
					if inputDataModified[i] > inputDataModified[j]:
						inversionCount += 1


		# Check if Odd or Even number of Inversions
		if inversionCount % 2 == 0:
			return True
		else:
			return False

#
# Get blank tile location (i,j) 
# Considering blank tile to be 0
#
# :type       inputData:  D array (list) 
# :param      inputData:  Input Data from User
#
# :returns:   The blank tile location [i,j]
# :rtype:     list 
#
def getBlankTileLocation(inputData):
	# Creating na empty list to save the blank tile location
	tileIndex = []
	rows = 3
	columns = 3
	for i in range(rows):
		for j in range(columns):
			if inputData[i][j] == 0:
				tileIndex.append(i)
				tileIndex.append(j)
	return tileIndex

#
# Define goal state
# Goal Matrix = 1 2 3
# 				4 5 6
# 				7 8 0
# 
# Checks if the current matrix combination is the goal state or not. 
# If yes retiurns True. Otherwise False
#
# :type       checkMatrix:  2D array (list)
# :param      checkMatrix:  The check matrix 
#
# :returns:   True is checkMatrix is same as goalMatrix. Otherwise False
# :rtype:     boolean
#
def goalState(checkMatrix):
	goalMatrix = [[1, 2, 3], [4, 5, 6], [7, 8, 0]]
	return (goalMatrix == checkMatrix)

#
# Define the 4 actions --> Left, Right, Up, Down
#
# :type       actionMatrix:  2D array (list)
# :param      actionMatrix:  The action matrix
#
# :returns:   if action is taken then returns the matrix on which the action is taken. Otherwise returns None
# :rtype:     2D array (list)
#
def moveLeft(actionMatrix):
	copyMatrix = copy.deepcopy(actionMatrix)

	# If 0 is in the left most block, then left is invalid
	if actionMatrix[0][0] != 0 and actionMatrix[1][0] != 0 and actionMatrix[2][0] != 0:
		[row, col] = getBlankTileLocation(actionMatrix)
		copyMatrix[row][col], copyMatrix[row][col-1] = copyMatrix[row][col-1], copyMatrix[row][col]
		return copyMatrix
	else:
		return None

def moveRight(actionMatrix):
	copyMatrix = copy.deepcopy(actionMatrix)

	# If 0 is in the right most block, then right is invalid
	if actionMatrix[0][2] != 0 and actionMatrix[1][2] != 0 and actionMatrix[2][2] != 0:
		[row, col] = getBlankTileLocation(actionMatrix)
		copyMatrix[row][col], copyMatrix[row][col+1] = copyMatrix[row][col+1], copyMatrix[row][col]
		return copyMatrix
	else:
		return None

def moveDown(actionMatrix):
	copyMatrix = copy.deepcopy(actionMatrix)

	# If 0 is in the bottom most block, then down is invalid
	if actionMatrix[2][0] != 0 and actionMatrix[2][1] != 0 and actionMatrix[2][2] != 0:
		[row, col] = getBlankTileLocation(actionMatrix)
		copyMatrix[row][col], copyMatrix[row+1][col] = copyMatrix[row+1][col], copyMatrix[row][col]
		return copyMatrix
	else: 
		return None

def moveUp(actionMatrix):
	copyMatrix = copy.deepcopy(actionMatrix)

	# If 0 is in the up most block, then up is invalid
	if actionMatrix[0][0] != 0 and actionMatrix[0][1] != 0 and actionMatrix[0][2] != 0:
		[row, col] = getBlankTileLocation(actionMatrix)
		copyMatrix[row][col], copyMatrix[row-1][col] = copyMatrix[row-1][col], copyMatrix[row][col]
		return copyMatrix
	else:
		return None

#
# Main function to solve the problem
# The problem is solved using BFS (Brute FOrce Search)
#
# :type       inputData:  2D array (list)
# :param      inputData:  The input data
#
def solvePuzzle(inputData):
	if isSolvable(inputData):
		print("Solvable\n Solving the Puzzle ...")

		# Empty queue created to save the visited combinations
		queue = []
		queuePointer = 0
		parentIndex = -1
		queue.append((inputData, parentIndex))
		nodeInfo = []

		while True:
			if goalState(queue[queuePointer][0]):
				print("Goal Reached")
				backTrack(queue, queuePointer, parentIndex)
				break

			# Saving the new combination and applying various actions on it
			# If node is not visited (after performing the action), it is added to the queue
			newConfig = moveLeft(queue[queuePointer][0])
			if (newConfig not in queue) and (newConfig is not None):
				queue.append((newConfig, queuePointer))

			newConfig = moveRight(queue[queuePointer][0])
			if (newConfig not in queue) and (newConfig is not None):
				queue.append((newConfig, queuePointer))

			newConfig = moveUp(queue[queuePointer][0])
			if (newConfig not in queue) and (newConfig is not None):
				queue.append((newConfig, queuePointer))

			newConfig = moveDown(queue[queuePointer][0])
			if (newConfig not in queue) and (newConfig is not None):
				queue.append((newConfig, queuePointer))

			# Collecting the node information at the given combination
			nodeInfo.append((queuePointer, queue[queuePointer][1]))
			queuePointer += 1

		# Saving the node info in "NodeInfo.txt" file
		np.savetxt("NodeInfo.txt", np.array(nodeInfo), fmt="%d")


	else:
		print("The given matrix is not solvable")

#
# Back Track the solution path to get the path from initial state to goal state
#
# :type       queue:  A list of all the visited conditions in the puzzle
# :param      queue:  The queue
#
def backTrack(queue, index, parentIndex):

	# Creating a list to save the combination while backtracking 
	storeArr = []

	# Backtrack till we reach the initial state
	while index != 0:
		storeArr.append(flatten(queue[index][0]))
		index = queue[index][1]
	storeArr.append(flatten(queue[0][0]))
	storeArr.reverse()

	# Saving the data in a "nodePath.txt" file
	nodePathFile = np.savetxt('nodePath.txt', storeArr, fmt="%d")

#
# Flatten the matrix to convert form 2D array to list
#
# :type       arrangeMatrix:  2D array (list)
# :param      arrangeMatrix:  The arrange matrix
#
# :returns:   the flattened matrix (1D array)
# :rtype:     array
#
def flatten(arrangeMatrix):
	flatMat = []

	# Converting 2D list to array 
	arrangeMatrixArr = np.array(arrangeMatrix)
	flatMat = arrangeMatrixArr.flatten('F')
	return flatMat



# Take user inpput in the form:
# row1 = 1 2 3
# row2 = 4 5 6
# row3 = 7 8 0
#
# :returns:   The input data collected from the user in a 2D array (list) form 
# :rtype:     list
#
def userInput():
	row1_inp = input("Enter Row 1: ").strip().split()
	row1 = list(map(int, row1_inp))
	row2_inp = input("Enter Row 2: ").strip().split()
	row2 = list(map(int, row2_inp))
	row3_inp = input("Enter Row 3: ").strip().split()
	row3 = list(map(int, row3_inp))

	inputData = []
	inputData.append(row1)
	inputData.append(row2)
	inputData.append(row3)

	return inputData

def main():
	"""
	Main Function for the program
	"""
	inputData = userInput()
	solvePuzzle(inputData)


if __name__ == "__main__":
	main()