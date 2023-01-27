# ROS2 (As Ease As Possible)

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
source yolobot/install/setup.bash

ros2 launch yolobot_gazebo yolobot_launch.py

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
<img src="https://user-images.githubusercontent.com/33729709/215048866-4bd22c58-7012-43d0-ab7f-ece3d95a66a3.png" width="500"/> 




<img src="https://user-images.githubusercontent.com/33729709/215048966-6b6b2aaa-4785-43e6-9e5e-b10ef4b59ded.png" width="500"/> 
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
<img src="https://user-images.githubusercontent.com/33729709/215049008-0c6938e7-0247-4921-b008-28402545af4e.png" width="500"/>  <img src="https://user-images.githubusercontent.com/33729709/215049083-a7eea250-d0c9-44e5-a4e7-46d899390b6f.png" width="500"/> 
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
<img src="https://user-images.githubusercontent.com/33729709/215049111-5b0f4626-9280-4d2f-958a-235ec214ace4.png" width="500"/>  <img src="https://user-images.githubusercontent.com/33729709/215049111-5b0f4626-9280-4d2f-958a-235ec214ace4.png" width="500"/> 
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
