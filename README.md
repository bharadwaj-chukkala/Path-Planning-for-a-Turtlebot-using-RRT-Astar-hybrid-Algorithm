# Path-Planning-for-a-Turtlebot-using-RRT-Astar-hybrid-Algorithm

**--- ENPM661 Path Planning for Autonomous Robots Final Project ---**

## Instructions for using the Repository

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

* Clone this repository, unzip and and paste it in your workspace.
* build the package using 'catkin_make'
* source it 'source devel/setup.bash'
* Run "export TURTLEBOT3_MODEL=burger" in the Terminal
* make the node executable, by navigating into the 'nodes' folder and running the command 'chmod +x task'
* launch the node and gazebo environment using 'roslaunch project_5 proj.launch'
  give clearance by typing in a value of "0.12"
* The node will start running in a couple of seconds
* After the path is visualized, close the window for the cmd_vel to be published.
  The turtlebot  will move to the goal in a minute.
