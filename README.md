# ROS2 (As Ease As Possible)

##### tested on
```bash
## ros distro
abc@xyz:~$ rosversion -d
humble

## ubuntu version
abc@xyz:~$ lsb_release -a
No LSB modules are available.
Distributor ID:	Ubuntu
Description:	Ubuntu 22.04.1 LTS
Release:	22.04
Codename:	jammy

## python version
abc@xyz:~$ python3 --version
Python 3.10.6


```
___
___

## my ros_ws
```bash

### steps for setting up
git clone https://github.com/bharath5673/ros_ws.git
cd ros_ws
colcon build
cd ..


```

___
___

## 1. turtlebot 


![Screenshot from 2023-01-27 14-14-57](https://user-images.githubusercontent.com/33729709/215048122-d9a7d4a3-f06e-495d-aa2d-6555d943df6c.png)

```bash

### steps for turtleot
source ros_ws/install/setup.bash
ros2 run turtlesim turtlesim_node & ros2 run my_robot_controller turtle_controller 

```

___
___
## 2. opencv-python 
___
![Screenshot from 2023-01-27 14-22-57](https://user-images.githubusercontent.com/33729709/215048416-1a3c2c45-ebf2-4578-b7c6-40b98e713b4b.png)



```bash

### steps for opencv

source ros_ws/install/setup.bash
ros2 run test_opencv run_test

```

___
___
## 3. mediapipe 
___
![ai-mediapipe-holistic](https://user-images.githubusercontent.com/33729709/215050919-cda2768e-d7bb-485b-b701-aeb61c51b50d.jpg)


```
pip install mediapipe
```


<div align="center">
<p>
<img src="https://user-images.githubusercontent.com/33729709/215048866-4bd22c58-7012-43d0-ab7f-ece3d95a66a3.png" width="400"/> 




<img src="https://user-images.githubusercontent.com/33729709/215048966-6b6b2aaa-4785-43e6-9e5e-b10ef4b59ded.png" width="400"/> 
</p>
<br>
</div>

```bash

### steps for facemesh

source ros_ws/install/setup.bash
ros2 run test_mediapipe facemesh_demo 

```
```bash

### steps for pose

source ros_ws/install/setup.bash
ros2 run test_mediapipe pose_demo 
```


<div align="center">
<p>
<img src="https://user-images.githubusercontent.com/33729709/215049008-0c6938e7-0247-4921-b008-28402545af4e.png" width="400"/>  <img src="https://user-images.githubusercontent.com/33729709/215049083-a7eea250-d0c9-44e5-a4e7-46d899390b6f.png" width="400"/> 
</p>
<br>
</div>

```bash

### steps for hands

source ros_ws/install/setup.bash
ros2 run test_mediapipe hands_demo 
```

```bash

### steps for holistic

source ros_ws/install/setup.bash
ros2 run test_mediapipe holistic_demo 
```

___
___
### 4.  yolov5
___
![test](https://user-images.githubusercontent.com/33729709/215051869-c1b9e0d0-9408-46d7-8756-3ca39a2c4576.jpg)

```
pip install yolov5
```

<div align="center">
<p>
<img src="https://user-images.githubusercontent.com/33729709/215049111-5b0f4626-9280-4d2f-958a-235ec214ace4.png" width="400"/>  <img src="https://user-images.githubusercontent.com/33729709/215049111-5b0f4626-9280-4d2f-958a-235ec214ace4.png" width="400"/> 
</p>
<br>
</div>


```bash

### steps for yolov5n

source ros_ws/install/setup.bash
ros2 run test_yolov5 yolov5n_demo 
```

```bash

### steps for yolov5s

source ros_ws/install/setup.bash
ros2 run test_yolov5 yolov5s_demo 
```


___
___
### 5.  Self-Driving-Car  
___

<div align="center">
<p>
<img src="https://github.com/noshluk2/ROS2-Self-Driving-Car-AI-using-OpenCV/blob/main/Images_videos/the_car.gif" width="400"/>  <img src="https://user-images.githubusercontent.com/33729709/216042261-ceb356c6-3e07-49d7-817e-ec28a41ed811.png" width="400"/> 
</p>
<br>
</div>


```
### steps for testing installation


## install dependancies
python3 -m pip install -r src/self_driving_car_pkg/requirements.txt



## once build you can run the simulation e.g [ ros2 launch (package_name) world(launch file) ] 

source ros_ws/install/setup.bash
source /opt/ros/humble/setup.bash
ros2 launch self_driving_car_pkg world_gazebo.launch.py


## To activate the SelfDriving Car

source ros_ws/install/setup.bash
source /opt/ros/humble/setup.bash
ros2 run self_driving_car_pkg computer_vision_node
```




<div align="center">
<p>
<img src="https://user-images.githubusercontent.com/33729709/216042810-cdb48e39-8aa4-4201-b8be-b77b8c51d7d7.png" width="400"/>  <img src="https://github.com/noshluk2/ROS2-Self-Driving-Car-AI-using-OpenCV/raw/main/Images_videos/Sat_Nav/1_localization.gif" width="400"/> 
</p>
<br>
</div>


````
### steps to run Self-Driving-Car


## Launch the maze_solving world in gazebo

source ros_ws/install/setup.bash
source /opt/ros/humble/setup.bash
ros2 launch self_driving_car_pkg maze_solving_world.launch.py



## in another terminal

source ros_ws/install/setup.bash
source /opt/ros/humble/setup.bash
ros2 run self_driving_car_pkg sdc_V2


````


for detailed explainations and tutorilas @ https://github.com/noshluk2/ROS2-Self-Driving-Car-AI-using-OpenCV



___
___
### 6.  rosbag and dataFrame  
___

![Screenshot from 2023-02-06 19-22-51](https://user-images.githubusercontent.com/33729709/216990025-51b69fb4-8097-4e61-9cda-b770822df93a.png)


```bash

### steps for rosbag turtle vel cmds

source ros_ws/install/setup.bash
ros2 run turtlesim turtlesim_node &  ros2 run test_turtle_bag turtlebot_for_rosbag 


### steps to control turtlebot 

source ros_ws/install/setup.bash
ros2 run turtlesim turtle_teleop_key


```
#### saving bag file
![Screenshot from 2023-02-06 19-21-34](https://user-images.githubusercontent.com/33729709/216990199-1db2606e-6979-496c-a690-8c4d1d4ea1eb.png)


```
### open turtlebot

source ros_ws/install/setup.bash
ros2 run turtlesim turtlesim_node 


### steps to rosbag play 

source ros_ws/install/setup.bash
ros2 bag play 'ros_ws/src/test_turtle_bag/test_turtle_bag/rosbag2_2023_02_06-18_46_50/rosbag2_2023_02_06-18_46_50_0.db3' -d 0.5


```



<div align="center">
<p>
<img src="https://user-images.githubusercontent.com/33729709/216971268-3275a8b1-b9bb-4710-8b66-7dad9c7eae91.png"/> 
</p>
<br>
</div>
@https://github.com/bharath5673/ros_ws/blob/main/src/test_turtle_bag/test_turtle_bag/rosbag.ipynb

___
___

### 7. ROS2 yolobot

```bash
## prerequisites

sudo apt install ros-humble-gazebo-ros
sudo apt-get install ros-humble-gazebo-msgs
sudo apt-get install ros-humble-gazebo-plugins

pip install yolov5
```
___


![Screenshot from 2023-01-28 17-31-40](https://user-images.githubusercontent.com/33729709/215265717-6c2092c0-4e0f-4cf3-bec8-51024dfb05bf.png)


```bash

### step for roslaunch
source /opt/ros/humble/setup.bash
source ros_ws/install/setup.bash

ros2 launch yolobot yolobot_launch.py

```

___
___


![Screenshot from 2023-01-28 16-55-02](https://user-images.githubusercontent.com/33729709/215265729-31d9f9b0-79ff-453e-9941-c39312bd6aa7.png)


 
```bash
### on new terminal


### step for yolobot detetcion 
source /opt/ros/humble/setup.bash
source ros_ws/install/setup.bash

python3 ros_ws/src/yolobot/yolobot_recognition/ros_recognition_yolo.py

 
```

for detailed explainations and tutorilas @ https://www.youtube.com/watch?v=594Gmkdo-_s&t=610s
___
___


### 8. PICO-W with MPU-6050 accelerometer and gyroscope module (EXCLUSIVE) <img src="https://media0.giphy.com/media/J19OSJKmqCyP7Mfjt1/giphy.gif" width="80" height="30" />

```bash
## prerequisites

sudo apt-get install ros-humble-imu-tools

```
___


![Screenshot from 2023-02-14 13-58-54](https://user-images.githubusercontent.com/33729709/218681454-8f2bb823-0ba0-42af-9a7b-e0cda1cbf405.png)


```bash

### step for simple testing imu sensors with random data 
source /opt/ros/humble/setup.bash
source ros_ws/install/setup.bash

ros2 ros2 run test_imu imu_simple_pub_sub 

```

 
```bash
### on new terminal for rviz

source ros_ws/install/setup.bash
rviz2 

now click on add create visualization By Topic /Imu/imu

```
 



#### Getting MPU-6050 Sensor Readings: Accelerometer, Gyroscope

<img src="https://user-images.githubusercontent.com/33729709/218695413-da3b14da-0146-408d-b602-6f4d288ee7a6.jpg" height="500" width="1080"/> 

flash ur pico-W on Thonny with https://github.com/bharath5673/ros_ws/blob/main/src/test_imu/pico_W/sketch1.py


```

### step to read imu MPU6050 data from pico-W and visualize on rviz2
source /opt/ros/humble/setup.bash
source ros_ws/install/setup.bash

ros2 ros2 run test_imu picoW_mpu6050 

```

```

### on new terminal for rviz

source ros_ws/install/setup.bash
rviz2 

```

___
___

## Cite

<details><summary> <b>Expand</b> </summary>

* [https://github.com/AlexeyAB/darknet](https://github.com/AlexeyAB/darknet)
* [https://github.com/ultralytics/yolov5](https://github.com/ultralytics/yolov5)
* [https://github.com/TexasInstruments/edgeai-yolov5/tree/yolo-pose](https://github.com/TexasInstruments/edgeai-yolov5/tree/yolo-pose)
* [https://github.com/noshluk2/ROS2-Self-Driving-Car-AI-using-OpenCV.git](https://github.com/noshluk2/ROS2-Self-Driving-Car-AI-using-OpenCV.git)
* [https://github.com/google/mediapipe.git](https://github.com/google/mediapipe.git)
* [https://github.com/ellenrapps/Road-to-Autonomous-Drone-Using-Raspberry-Pi-Pico.git](https://github.com/ellenrapps/Road-to-Autonomous-Drone-Using-Raspberry-Pi-Pico.git)
* [https://github.com/ros2/ros2_documentation.git](https://github.com/ros2/ros2_documentation.git)
* [https://github.com/raspberrypi](https://github.com/raspberrypi](https://github.com/raspberrypi)
* [https://github.com/NVIDIA](https://github.com/NVIDIA)
* [https://github.com/opencv/opencv](https://github.com/opencv/opencv)
  
  
</details>

