This is the ReadMe file for running ENPM661: Project_1: Solving the 8-Puzzle Problem

## **Problem Statement:** 
To solve the puzzle by moving the blank tile such that the final combination of the matrix looks like this:
  
1 2 3 

4 5 6

7 8 0

where "0" is the blank tile


## **Approach:**
We need to reach the goal state from the given initial state.

The puzzle is solved by applying Brute Force Search (BFS) algorithm. 

In this case, all the possible combinations are considered, without repeating any of the combinations, to check for the goal state.

Backtracking is used to check the steps taken to reach from initial state to goal state.

## **Running the Code:**

Run the code on the commandline using the follwoing: `python3 main.py`

The user will then be prompted to give the input grid. Enter the grid in the given format.

`Enter row1 = 1 2 0`

 `Enter row2 = 4 5 3`
 
 `Enter row3 = 7 8 6`

The code first tells if the given inpput grid is solvable or not. 

It then prompts if the goal state is reached.

## ** Output Files:**

The output is saved in two files:

  * `nodePath.txt` which gives the path followed to go from initial state to goal state
  *  `nodeInfo.txt` which stores the node index and the parent node index for reference. 

## **Steps taken to solve the 8-Puzzle:**

To check the steps used to solve the problem type the following command on the terminal: 

`python3 plot_path.py`

 
