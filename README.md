# Kobuki
Sources:

https://github.com/kobuki-base

https://github.com/ros-controls/gz_ros2_control/tree/jazzy

https://yalmip.github.io/

https://sedumi.ie.lehigh.edu/

# Testing ROS Nav2
To test ROS NAVIGATION, uncomment line 145 of kobuki.launch.py
#### GAZEBO/RVIZ LAUNCH
     ros2 launch kobuki_descripcion kobuki.launch.py
#### SLAM LAUNCH
     ros2 launch slam_toolbox online_async_launch.py slam_params_file:=./src/kobuki_ros/kobuki_description/config/slam_parameters.yaml use_sim_time:=true
#### NAV2 LAUNCH
     ros2 launch nav2_bringup navigation_launch.py use_sim_time:=true
     
# Testing LMIs-based and LQR controllers for path tracking
#### GAZEBO/RVIZ LAUNCH
     ros2 launch kobuki_descripcion kobuki.launch.py
#### CONTROL RUN
     ros2 run kobuki_controllers LMIs
![image](https://github.com/MOBILAB-UDESC/Kobuki-ROS2-Jazzy/blob/main/src/kobuki_ros/kobuki_controllers/Example1_path_tracking.png)