# Implementation-of-A-Star-algorithm-on-a-differential-drive-non-holonomic-robot

![](Vrep_turtlebot.gif)

Python Implementation of A* algorithm on a differential drive (non-holonomic) robot:

Implemented A* algorithm to find a path between start and end point on a fixed map for a TurtleBot (Radius + clearance=20cm)

I used Minkowski Sum method to expand the obstacles and the boundary of the given map.

Considered the workspace as a 8 connected space, two different speeds for straight moves and 3 variations of left and right turns with cost to come and cost to goal being euclidean distances.

I used Half-planes and semi-algebraic models to represent the obstacle space.Illustrated optimal path generation as well as node exploration animation between start and goal point using a OpenCV interface and simulated the robot in V-REP environment.
