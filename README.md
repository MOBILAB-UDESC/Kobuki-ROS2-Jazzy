# Kobuki
Sources:

https://github.com/kobuki-base

https://github.com/ros-controls/gz_ros2_control/tree/jazzy

https://yalmip.github.io/

https://sedumi.ie.lehigh.edu/

# Testing ROS Nav2
#### GAZEBO/RVIZ LAUNCH + SLAM
     ros2 launch kobuki_description kobuki_launch.py world_name:=playground.sdf slam_type:=slam
#### NAV2 LAUNCH
     ros2 launch nav2_bringup navigation_launch.py use_sim_time:=true

# Testing LMIs-based and LQR controllers for path tracking
#### SIMULATION
     ros2 launch kobuki_description kobuki_launch.py
#### REAL ROBOT
     ros2 launch kobuki_description kobuki_launch.py use_sim_time:=false
#### CONTROL RUN
     ros2 run kobuki_controllers DStabDelay
![image](https://github.com/MOBILAB-UDESC/Kobuki-ROS2-Jazzy/blob/main/src/kobuki_ros/kobuki_controllers/Example1_path_tracking.png)