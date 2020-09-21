# Path-Planning-using-A_Star
Implement A* Algorithm to find the shortest path between start and endpoint.

# ENPM661_Proj3_Phase2
A* implementation for Rigid Robot

The main file is Astar_rigid.py

## Dependencies
Numpy
Matplotlib
Shapely
Heapq
time

## Instructions to run A* for a Rigid Robot
```python
$ git clone https://github.com/revati-naik/Path-Palnning-using-A_Star.git
$ python codes/Astar_rigid.py
```
The user will then be prompted for various inputs.
Once the program finishes, the optimal path to the goal is displayed.
Consider the example below.

```
Enter starting coordinates (x y) and orientation: 50 30 60
Enter goal coordinates (x y): 150 150
Enter the robot radius: 1
Enter the clearance: 1
Enter the robot step_size: 1

Reached Goal!
Current node:---
current_coords:	(149.5954988232819, 149.34678751731778)
parent_coords:	(149.0954988232819, 148.48076211353333)
orientation:	30
movement_cost:	190.0
goal_cost:	0.7683148765305786
Goal node:---
current_coords:	(150.0, 150.0)
parent_coords:	None
orientation:	None
movement_cost:	None
goal_cost:	0
Time to run A*: 765.2316 seconds
```

The file `a_star_rigid.mp4` shows the animation for the exploration of the search space and the final chosen optimal path.
