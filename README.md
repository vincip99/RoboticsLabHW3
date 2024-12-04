# RoboticsLabHW3

## :package: About

## :hammer: Build

## :white_check_mark: Usage
Run the gazebo world with the spherical object and detect it with openCV:

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

Run the gazebo world with the aruco marker and use the controllers:

ros2 launch iiwa_bringup iiwa.launch.py use_sim:=true use_vision:=true command_interface:="velocity" robot_controller:="velocity_controller"

ros2 launch aruco_ros single.launch.py marker_size:=0.1 marker_id:=201

ros2 run ros2_kdl_package ros2_kdl_vision_control