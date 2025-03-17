# Kobuki
Source: https://github.com/kobuki-base

# Testing ROS Nav2
ros2 launch kobuki_descripcion kobuki.launch.py
ros2 launch slam_toolbox online_async_launch.py slam_params_file:=./src/kobuki_ros/kobuki_description/config/slam_parameters.yaml use_sim_time:=true
ros2 launch nav2_bringup navigation_launch.py use_sim_time:=true

# Testing LMIs-based and LQR controllers for path tracking
ros2 run ...
