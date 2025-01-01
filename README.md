# RoboticsLabHW3
Homework 3 for Robotics Lab 2024/2025

## :package: About
Implement a vision-based task with OpenCV and AruCo markers

## :hammer: Build
First build all the packages by using:

```
colcon build --packages-select iiwa_description iiwa_description iiwa_bringup ros2_kdl_package ros2_opencv
```
In each terminal you open, source the install directory:
```
source install/setup.bash
```

## :white_check_mark: Usage
### Run the gazebo world with the spherical object and detect it with openCV:

Launch the simulation with the command
```
ros2 launch iiwa_bringup iiwa.launch.py use_sim:=true use_vision:=true gz_world:=spherical_object_world.world
```

Run the detector node with the command
```
ros2 run ros2_opencv ros2_opencv_node
```

Visualize the images with
```
run rqt_image_view rqt_image_view
```

The detected image is on the /processed_image topic

### Run the gazebo world with the aruco marker and use the controllers:

Positioning Controller
```
ros2 launch iiwa_bringup iiwa.launch.py use_sim:=true use_vision:=true command_interface:="velocity" robot_controller:="velocity_controller"
```

In another terminal launch the AruCo marker detector
```
ros2 launch aruco_ros single.launch.py marker_size:=0.1 marker_id:=201
```

In another terminal, run the control node
```
ros2 run ros2_kdl_package ros2_kdl_vision_control --ros-args -p task:="positioning" -p cmd_interface:="velocity"
```

Velocity controller with look at point controller:

```
ros2 launch iiwa_bringup iiwa.launch.py use_sim:=true use_vision:=true command_interface:="velocity" robot_controller:="velocity_controller"
```

In another terminal launch the AruCo marker detector
```
ros2 launch aruco_ros single.launch.py marker_size:=0.1 marker_id:=201
```

In another terminal, run the control node
```
ros2 run ros2_kdl_package ros2_kdl_vision_control --ros-args -p cmd_interface:="velocity" -p task:="look-at-point"
```

Effort controller with inverse dynamics joint space look at point controller:

```
ros2 launch iiwa_bringup iiwa.launch.py use_sim:=true use_vision:=true command_interface:="effort" robot_controller:="effort_controller"
```

In another terminal launch the AruCo marker detector
```
ros2 launch aruco_ros single.launch.py marker_size:=0.1 marker_id:=201
```

In another terminal, run the control node
```
ros2 run ros2_kdl_package ros2_kdl_vision_control --ros-args -p cmd_interface:="effort" -p cmd_type:="jnt_id"
```

Effort controller with inverse dynamics operational space look at point controller:

```
ros2 launch iiwa_bringup iiwa.launch.py use_sim:=true use_vision:=true command_interface:="effort" robot_controller:="effort_controller"
```

In another terminal launch the AruCo marker detector
```
ros2 launch aruco_ros single.launch.py marker_size:=0.1 marker_id:=201
```

In another terminal, run the control node
```
ros2 run ros2_kdl_package ros2_kdl_vision_control --ros-args -p cmd_interface:="effort" -p cmd_type:="op_id"
```