# Lab 4 - Rapidly-exploring Random Tree (RRT)

This repo is tested with python 2.7 and pybullet 2.5.6.



## Part 1 - RRT

### Usage

```
$ python demo.py
```

### Methods

```
def rrt(max_iter, start_conf, end_conf):
"""
Grows trees from the starting point in the direction of a randomly chosen point.
Randomly chosen point has a bias of 5% towards the goal.
If any branch in any tree is 0.075 distance from the goal then we are done.

returns a valid path configuration from start to goal
"""



def create_step(p1,p2):
"""
Step size function for path rrt and birrt. 
Utilizes direction vector to create new point incrementally closer to p2 from p1.
Delta variable defines granularity of tree.

returns a vector
"""



def extend_rrt(q_near, q_rand):
"""
Utilizes create_step function to get a new vector in the direction of q_rand.
If that new vector is not a collision we write it's debug line and return it.

returns q_near and q_new if there are no collisions
"""



def dist_fn(p1, p2):
"""
Basic distance function.

returns distance from vector p1 to vector p2
"""
```

### Video

https://youtu.be/q6B1-j1vQrA




## Part 2 - Bidirectional RRT

```
python demo.py --birrt
```

```
def birrt(max_iter, start_conf, end_conf):
"""
Grows tree from the starting point in the direction of a randomly chosen point.
Then grows tree from goal in the direction of the step size vector from the opposing
tree, and add step size for goal tree.
Switch trees and repeat.
If any branch on either tree is 0.1 distance from the the other tree then we are done.


returns a valid path configuration from start to goal
"""



def create_step(p1,p2):
"""
Step size function for path rrt and birrt. 
Utilizes direction vector to create new point incrementally closer to p2 from p1.
Delta variable defines granularity of tree.

returns a vector
"""



def extend_rrt_b(q_near, q_rand):
"""
Utilizes create_step function to get a new vector in the direction of q_rand.
If that new vector is not a collision we write it's debug line with the
right color according to what tree we are growing and return the new vector.


returns q_near and q_new if there are no collisions
"""



def dist_fn(p1, p2):
"""
Basic distance function.

returns distance from vector p1 to vector p2
"""
```

### Video

https://youtu.be/rgwvj3IqJlw




## Extra Credit - Bidirectional Path Smoothing

### Usage

```
$ python demo.py --birrt --smoothing
```

### Methods

```
def birrt_smoothing(smooth_iter, max_iter, start_conf, end_conf):
"""
Uses birrt function to create path_conf. 

Loops N times. Picks two random points from path_conf, and steps along the straight
path between those points using create_smooth_step. If that straight path does not 
have a collision, it snips all confs between the two randomly chosen points in 
path_conf and replaces with the straight path.

returns a new path conf
"""



def dist_fn(p1, p2):
"""
Basic distance function.

returns distance from vector p1 to vector p2
"""



def create_smooth_step(p1,p2):
"""
Step size function for path smoothing. 
Utilizes direction vector to create new point incrementally closer to p2 from p1.
Delta variable defines granularity of smoothed path.

returns a vector
"""
```

### Video

https://youtu.be/lYsp-pqf6qE


