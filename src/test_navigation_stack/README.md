# ROS2 Navigation Stack
___
___


##### step 1

```bash

### steps for setting up
sudo apt install ros-humble-navigation2 ros-humble-nav2-bringup ros-humble-turtlebot3* ros-humble-turtlebot3-simulations 
sudo apt install ros-humble-rmw-cyclonedds-cpp

sudo nano ~/.bashrc
export TURTLEBOT3_MODEL=waffle
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

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

