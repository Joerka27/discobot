## AR Tags

if not yet installed:

```
sudo apt install ros-kinetic-image-common
sudo apt install ros-kinetic-image-pipeline
sudo apt install ros-kinetic-ar-track-alvar
```

---

### Raspberry Cam

try this?

https://github.com/UbiquityRobotics/raspicam_node

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

```
rosrun camera_calibration cameracalibrator.py --size 7x7 --square 0.0095 image:=/usb_cam/image_raw camera:=/usb_cam --pattern circles
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
