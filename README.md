# EE346-Lab6

Launch roscore on Remote PC.
```
roscore
```
Trigger the camera and bringup the turtlebot on SBC.
```
roslaunch turtlebot3_bringup turtlebot3_robot.launch
roslaunch turtlebot3_autorace_traffic_light_camera turtlebot3_autorace_camera_pi.launch
```

Run a intrinsic camera calibration launch file on Remote PC.
```
$ export AUTO_IN_CALIB=action
$ export GAZEBO_MODE=false
$ roslaunch turtlebot3_autorace_traffic_light_camera turtlebot3_autorace_intrinsic_camera_calibration.launch
```
<!-- Run a extrinsic camera calibration launch file on Remote PC.
```
$ export AUTO_EX_CALIB=action
$ roslaunch turtlebot3_autorace_traffic_light_camera turtlebot3_autorace_extrinsic_camera_calibration.launch
``` -->

Run the lane follower node
```
cd ~/catkin_ws/src/EE346-Lab6/src
python turtlebot3_lane_follower.py
```

View calibrated images
`rqt_image_view`

Change ISO if the frame is shaking

`rosrun rqt_reconfigure rqt_reconfigure`