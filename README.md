# Path-Planning-for-a-Turtlebot-using-RRT-Astar-hybrid-Algorithm

## Introduction

The RRT path planning algorithm is quite popular for its speed and implementation. The only downside to RRT is the optimality in the planning process, as the nodes are selected at random, different planning costs are obtained due to the metric function. Therefore an improvised heuristic RRT-A* algorithm is proposed for a mobile robot motion planning with non-holonomic constraints. This algorithm makes the performance
optimal by introducing the cost of the A Start algorithm into the RRT algorithm. A mobile robot such as a turtle bot with a non-holonomic constraints has been used to implement this algorithm where simulation results have shown that the Manhattan heuristic information function based RRT-A* planning algorithm is better than the other improved RRT algorithms in the optimization path and computational cost.

---

### Contents

```
├───output
│   ├───output images
│   └───Turtlebot Simulation.mp4
├───project_5
├───661_project5_final_report.pdf
├───LICENSE
└───README.md
```

### Pre Requisites

* Ubuntu 18.04
* ROS Melodic
* Gazebo 9.1
* Turtlebot3 Packages

---

### Dependencies

* numpy
* matplotlib.patches
* math
* rospy
* time
* heapq
* random
* sys
* pygame
* Turtlebot3 Packages

---

### Installing Turtlebot Package

**Paste the following commands line by line**

- ``mkdir planning_ws/src``
- ``catkin_make``
- ``source devel/setup.bash``
- ``git clone https://github.com/ROBOTIS-GIT/turtlebot3.git``
- ``git clone https://github.com/ROBOTIS-GIT/- turtlebot3_msgs.git``
- ``git clone https://github.com/ROBOTIS-GIT/turtlebot3_simulations.git``
- ``cd  ../ && catkin_make``
- ``echo "source ~/planning_ws/devel/setup.bash" >> ~/.bashrc``
- ``source ~/.bashrc``

---

### Running the package

* Clone this repository
* `git clone`
* unzip the cloned repository
* paste the `project_5` package in your catkin workspace.
* build the package using `catkin_make`
* source it `source devel/setup.bash`
* Run `export TURTLEBOT3_MODEL=burger` in the Terminal
* make the node executable, by navigating into the `nodes` folder and running the command
* `chmod +x task` here task is the name  of the executable.
* launch the node and gazebo environment using
* `roslaunch project_5 proj.launch`
* give clearance by typing in a value of `0.12`
* The node will start running in a couple of seconds
* After the path is visualized, close the window for the cmd_vel to be published.
* The turtlebot  will move to the goal in a minute.

---

## Results

|                  | RRT-Euclidean | RRT-Manhattan | Hybrid-RRT-Euclidean | Hybrid-RRT-Manhattan |
| ---------------- | ------------- | ------------- | -------------------- | -------------------- |
| Sparse Obstacles |  <img width="200" height="200" src="https://github.com/bharadwaj-chukkala/Path-Planning-for-a-Turtlebot-using-RRT-Astar-hybrid-Algorithm/blob/main/outputs/output%20images/RRT_euclid_sparse.png">             |      <img width="200" height="200" src="https://github.com/bharadwaj-chukkala/Path-Planning-for-a-Turtlebot-using-RRT-Astar-hybrid-Algorithm/blob/main/outputs/output%20images/RRT_manhat_sparse.png">       |         <img width="200" height="200" src="https://github.com/bharadwaj-chukkala/Path-Planning-for-a-Turtlebot-using-RRT-Astar-hybrid-Algorithm/blob/main/outputs/output%20images/RRT_heur_euclid_sparse.png">             |        <img width="200" height="200" src="https://github.com/bharadwaj-chukkala/Path-Planning-for-a-Turtlebot-using-RRT-Astar-hybrid-Algorithm/blob/main/outputs/output%20images/RRT_heur_manhat_sparse.png">              |
| Dense Obstacles  |      <img width="200" height="200" src="https://github.com/bharadwaj-chukkala/Path-Planning-for-a-Turtlebot-using-RRT-Astar-hybrid-Algorithm/blob/main/outputs/output%20images/RRT _euclid_dense.png">        |             <img width="200" height="200" src="https://github.com/bharadwaj-chukkala/Path-Planning-for-a-Turtlebot-using-RRT-Astar-hybrid-Algorithm/blob/main/outputs/output%20images/RRT_manhat_dense.png">    |            <img width="200" height="200" src="https://github.com/bharadwaj-chukkala/Path-Planning-for-a-Turtlebot-using-RRT-Astar-hybrid-Algorithm/blob/main/outputs/output%20images/RRT_heur_euclid_dense.png">          |           <img width="200" height="200" src="https://github.com/bharadwaj-chukkala/Path-Planning-for-a-Turtlebot-using-RRT-Astar-hybrid-Algorithm/blob/main/outputs/output%20images/RRT_heur_manhat_dense.png">         |
