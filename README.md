# Person tracking with scout mini robot 
This repository is an algorithm for tracking person using `YOLOv5`. The robot used is a `Scout Mini`, and `ROS1` was used. 

## Quick start
- Connect can communication
```
candump can0
```
- Bringup scout mini
```
roslaunch scout_bringup scout_mini_robot_base.launch
```
- Run tracking node
```
roslaunch ultralytics_ros tracker.launch
```

- Run usb cam (before run this, install usb_cam package)
```
 roslaunch usb_cam usb_cam.launch
 ```

## Result 
![person tracking](./객체추종.gif)
