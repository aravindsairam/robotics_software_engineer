# Assignments for Module #2 : ROS2 2D Turlesim Simulation
## Assignment 1: Custom Nodes and Launch Files

### Move in user input radius of circle

To Run:
```
$ cd ~/assignment_ws
$ colcon build && source install/setup.bash
$ ros2 run draw_circle 0.5 # where 0.5 is the user input radius
```

### Move the turtle in simple/archimedean spiral (just for my understanding)
```
$ cd ~/assignment_ws
$ colcon build && source install/setup.bash
$ ros2 run draw_spiral
```

### Move the turtle in log spiral
```
$ cd ~/assignment_ws
$ colcon build && source install/setup.bash
$ ros2 launch module_2_assignment unique_pattern.launch.py
```


## Assignment 2: Exploring Turtlesim Services and Parameters


### Spawn 5 Turtlebots with 1 single launch file in Diagonal from top to bottom | Drive middle 3 turtles back and forth continuosly

```
$ cd ~/assignment_ws
$ colcon build && source install/setup.bash
$ ros2 launch module_2_assignment spawn_multi_turtle.launch.py
```

### Modify the behavior of the Turtlesims using ROS2 parameters | change the speed of the turtles

```
$ cd ~/assignment_ws
$ colcon build && source install/setup.bash
$ ros2 launch module_2_assignment control_turtle_speeds.launch.py

# To change the speeds dynamically

# First check the node list
$ ros2 node list

# Get the node params list (example below)
$ ros2 param list /speed_change_1

# get the specific param value in a node
$ ros2 param get /speed_change_1 linear_x

# set the linear veloicty param of node /speed_change_1
$ ros2 param set /speed_change_1 linear_x 0.5

# set the angular veloicty param of node /speed_change_1
$ ros2 param set /speed_change_1 angular_z 0.5

```