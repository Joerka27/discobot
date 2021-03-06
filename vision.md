## AR Tags

if not yet installed:

```
sudo apt install ros-kinetic-image-common
sudo apt install ros-kinetic-image-pipeline
sudo apt install ros-kinetic-ar-track-alvar
```

---

### Raspberry Cam

First, enable hardware support for the camera as described here:
https://www.raspberrypi.org/documentation/configuration/camera.md

```
sudo raspi-config

# test:
raspistill -v -o test.jpg
```

Clone and build the ROS node as described here:
https://github.com/UbiquityRobotics/raspicam_node

Launch the node:
```
roslaunch raspicam_node camerav1_1280x720.launch
```

---

### usb_cam (for Webcams)

```
cd < catkin_ws >/src
git clone https://github.com/bosch-ros-pkg/usb_cam.git
cd ..
catkin_make
source devel/setup.bash
```

now `roslaunch usb_cam usb_cam-test.launch` should work

---

## Calibrate

* launch camera node before
* change parameters if needed...

Raspi:
```
rosrun camera_calibration cameracalibrator.py --size 7x7 --square 0.01875 image:=/raspicam_node/image/compressed camera:=/raspicam_node --pattern circles
```

webcam:
```
rosrun camera_calibration cameracalibrator.py --size 7x7 --square 0.01875 image:=/usb_cam/image_raw camera:=/usb_cam --pattern circles
```

* ...calibration GUI opens...
* move checkerboard in different poses
* press "calibrate" once it appears
* press "commit" to send the calibration to the camera node

---

## Marker Tracking

* launch camera node before
* adapt marker size, camera topic and frame in launch file if needed

```
roslaunch ar_tag_raspi.launch
```

--> publishes 3D pose in `/ar_pose_marker`

docs:

http://wiki.ros.org/ar_track_alvar

### Pose

published as `tf` with name `ar_marker_x` (x=ID of th tag)
