<br>
<img src="https://media0.giphy.com/media/J19OSJKmqCyP7Mfjt1/giphy.gif" width="80" height="30" /> 

# ROS2 Navigation Stack

___
___

## Creating Maps

##### step 1

```bash

### steps for setting up
sudo apt install ros-humble-navigation2 ros-humble-nav2-bringup ros-humble-turtlebot3* ros-humble-turtlebot3-simulations 
sudo apt install ros-humble-rmw-cyclonedds-cpp

sudo sh -c "echo 'export TURTLEBOT3_MODEL=waffle' >> ~/.bashrc"
sudo sh -c "echo 'export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp' >> ~/.bashrc"
```
___


##### step 2 
```bash

### steps for creating maps

## launch turtlebot3 on gazebo 
ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py 

## now start turtlebot3_cartographer to map env
ros2 launch turtlebot3_cartographer cartographer.launch.py 

## control robot via telop keys
ros2 run turtlebot3_teleop teleop_keyboard 

## after SLAM allover save maps
mkdir turtlebot3_maps
ros2 run nav2_map_server map_saver_cli -f turtlebot3_maps/map_test_1

```

```

## check weather maps saved
ls turtlebot3_maps
map_test_1.pgm  map_test_1.yaml


```
___


##### step 3
```bash

### steps for loading maps and navigate turtlebot
ros2 launch turtlebot3_navigation2 navigation2.launch.py map:=turtlebot3_maps/map_test_1.yaml 


```
select and play according..

___
___


## WayPoint Navigation

Waypoint navigation in ROS2 refers to the process of defining a set of pre-defined locations or "waypoints" for a robot to navigate through in a specific sequence. This involves setting up a list of goals for the robot to reach and then using a navigation algorithm to plan the robot's path through the waypoints uisng maps.

 
##### step 1 
```bash
## launch turtlebot3 on gazebo 
ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py 
```

##### step 2 
```bash
## now launch gazebo rtab using saved map on new tab
ros2 launch turtlebot3_navigation2 navigation2.launch.py use_sim_true:=True map:='turtlebot3_maps/map_test_1.yaml'  
```

#### Initialize pose
reinit robot location on rtab looking into robot orientations on gazebo sim to sync env
![output3](https://github.com/bharath5673/ros_ws/blob/main/src/navigation_tb3/output1.gif)


#### setting a goal 
set waypoint to navigate robot
![output3](https://github.com/bharath5673/ros_ws/blob/main/src/navigation_tb3/output2.gif)  


#### setting multi goals
set multiwaypoints to nav through multiple locations
![output3](https://github.com/bharath5673/ros_ws/blob/main/src/navigation_tb3/output3.gif)   


##  Commander API
The Commander API provides a set of functions that enable the robot to perform navigation tasks such as setting the robot's initial pose, setting a goal position, and canceling a navigation task in one single run ;)
![output3](https://github.com/bharath5673/ros_ws/blob/main/src/navigation_tb3/output4.gif) 


