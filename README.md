# discobot

## Remember to prepare before launching

```sh
# get access to USB serial port
sudo chown discobot:discobot /dev/ttyACM0

# put these in your ~/.bashrc
source /opt/ros/kinetic/setup.bash
source ~/catkin_ws/devel/setup.bash

# if not running on localhost
export ROS_MASTER_URI=http://x.x.x.x:11311
export ROS_IP=M.Y.I.P
```

## Launch dat bot
Launches the serial connection and the simple battle bot node.
```sh
roslaunch discobot robot.launch
```
Also launch camera and marker detection:
```sh
roslaunch discobot robot_cam_ar.launch
```
