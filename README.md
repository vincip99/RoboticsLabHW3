# RoboticsLabHW3

ros2 launch iiwa_bringup iiwa.launch.py use_sim:=true use_vision:=true command_interface:="velocity" robot_controller:="velocity_controller"

ros2 launch aruco_ros single.launch.py marker_size:=0.1 marker_id:=201

ros2 run ros2_kdl_package ros2_kdl_vision_control