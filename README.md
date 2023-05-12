# Turtle chaser (ROS exercise)

ROS package with launch file that spawns two turtles. First turtle is controlled from keyboard, second turtle automaticaly chase first one.

## Showcase

![Chase turtlesim example](screenshots/chase_turtlesim_example.png)

## How to start

```bash
catkin_make  # first time after clone
source devel/setup.bash
roslaunch turtle_chaser chaser.launch
```

## Start rqt_graph node

```bash
rosrun rqt_graph rqt_graph
```
