# Path-Planning-using-A_Star (with non-holonomic constraints)
Implementation of A* Algorithm to find the shortest path between start and endpoint.

# ENPM661_Proj3_Phase3
A* implementation for Rigid Robot (turtlebot3 - Burger) with non-holonomic constraints

The main file is `Astar_rigid.py`

## Dependencies
Numpy

Matplotlib

Shapely

Heapq

time

## Instructions to run A* for a Rigid Robot
```python
$ git clone https://github.com/revati-naik/Path-Planning-using-A_Star.git
$ cd Path-Planning-using-A_Star/phase_3
$ python codes/Astar_rigid.py
```
The user will then be prompted for various inputs.
Once the program finishes, the optimal path to the goal is displayed.
Consider the example below.

```
Enter starting coordinates (x y) and orientation: -4 -4 10
Enter goal coordinates (x y): 4 3
Enter rpm1 and rpm2: 20 20
Enter the clearance: 0.2

-----------------------
Start Node:
current_coords:	(-4.0, -4.0)
parent_coords:	None
orientation:	0
Action:		None
movement_cost:	0
goal_cost:	10.63014581273465
-----------------------
Reached Goal!
Final Node:
current_coords:	(3.0140359857634746, 4.048077894628668)
parent_coords:	(2.72917156989548, 4.21076446688913)
orientation:	45.9260311266
Action:		(20.0, 0)
movement_cost:	14.63000000000007
goal_cost:	0.05008485647656169
------------------------
Time to run A*: 284.823743 seconds
('Number of visited nodes:', 5128)
('Number of nodes in path:', 26)

```

The file `a_star_rigid_nonholonomic_constraints.mp4` shows the animation for the exploration of the search space and the final chosen optimal path.
