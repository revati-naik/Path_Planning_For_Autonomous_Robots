# Path-Planning-using-A_Star - TurtleBot-3 
Implementation of path planning using A* Algorithm with the non-holonomic constraints, to find the shortest path between start and endpoint on turtlebot3.
---


## Overview

Agenda of this simple implementation is the demonstration the working of a path planning algorithm on a simulation environment -- in our case, a turtlebot3.

The launch file invokes the gazebo simulation node, the turtlebot3 node, and the publisher to move the turtlebot.

## Dependencies

This package has been tested in a system with following dependencies.
- Ubuntu 18.04 LTS
- ROS-Melodic
- Gazebo version 9.12.0

## Build Instructions
1) Clone the repository:
```
$ source /opt/ros/kinetic/setup.bash
$ git clone https://github.com/revati-naik/Path-Planning-using-A_Star.git
```

3) Build and source the package:
```
$ mkdir -p ~/catkin_ws/src
$ cp -r Path-Planning-using-A_Star/phase_4 ~/catkin_ws/src/
$ cd ~/catkin_ws/src
$ catkin_make
$ source devel/setup.bash
```

## Run Instructions

There are two launch files of interest. One that runs on the precomputed path and one that computes the optimum path just before running the robot.

# Case 1:
*Launch file:* `./launch/turtlebot_astar_from_npy.launch`
The main parameters to specify here are the start position, and the npy file that stores the optimum computed path.

Example:
```
roslaunch turtlebot_astar turtlebot_astar_from_npy.launch
```


# Case 2:
*Launch file:* `./launch/turtlebot_astar_planner.launch`
The main parameters to specify here are the start position, the goal position, the rpms and the clearance.

1) Example:
```
$ roslaunch turtlebot_astar turtlebot_astar_planner.launch start_x:=-4 start_y:=-4 start_theta:=20 goal_x:=4 goal_y:=2 rpm_1:=10 rpm_2:=20 clearance:=0.2

-----------------------
Start Node:
current_coords:	(-4, -4)
parent_coords:	None
orientation:	20
Action:		None
movement_cost:	0
goal_cost:	10.0
-----------------------
Reached Goal!
Final Node:
current_coords:	(1.9424025646663647, 3.811005181507038)
parent_coords:	(1.4468884047195838, 3.4880127270350316)
orientation:	20.0
Action:		(20, 10)
movement_cost:	11.286
goal_cost:	0.197576582555
------------------------
Time to run A*: 912.565353 seconds
('Number of visited nodes:', 9065)
('Number of nodes in path:', 25)
Cannot plot the curve for this node.
Done with plots.
ROS shutdown request
```
These parameters correspond to the second start-goal position configuration.

This file launches the gazebo environment, takes in the required arguments for the turtlebot3 simulation robot, spawns the robot, and launches the publisher to plan the path and move the robot accordingly.


## Outputs
`Phase4_video1` and `Phase4_video2` show the output of the given start-goal position configurations.