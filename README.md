# ros_decawave
Positioning System based on Decawave's DWM1001 Ultra Wide Band transceivers.

## Dependencies

### Python Dependencies
```
$ pip install serial
$ pip install struct
```

### ROS Dependencies
```
$ apt install ros-kinetic-hector-trajectory-server
```

## Tutorial
### Pre-Setup
Setup the tag and the anchors using the Android app, and connected the tag using an USB cable to PC.

### Download and Compilation
Download this repo into the ros workspace and compile:
```
$ cd src/
$ git clone https://github.com/verlab/ros_decawave.git
$ cd ..
$ catkin_make ## or catkin build
```

### Using
Run the launch:
```
$ roslaunch ros_decawave decawave_driver.launch
```
[![IMAGE ALT TEXT HERE](https://img.youtube.com/vi/ieaP79FDLC0/0.jpg)](https://www.youtube.com/watch?v=ieaP79FDLC0)

### Topics
```
$ rostopic list
/pose # tag position
/rosout
/rosout_agg
/status # anchors status
/syscommand
/tf 
/tf_static
/trajectory # tag path
```

