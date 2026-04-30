The UR5e implementation uses a prebuilt Docker image running on ROS2 Humble Hawksbill and Gazebo classic. 

To start, run the container 
docker run -it --name ur5yt --net=host -e DISPLAY=$DISPLAY -e QT_X11_NO_MITSHM=1 -e RMW_IMPLEMENTATION=rmw_cyclonedds_cpp -e ROS_DOMAIN_ID=0 -v /tmp/.X11-unix:/tmp/.X11-unix:rw -v /dev/dri:/dev/dri -v ~/workspaces/ur5_yt_docker/src:/workspaces/app_ws/src learnroboticswros/ur5_gripper_camera_sim:gui

For each terminal you eventually need, you will need to type 
xhost +si:localuser:root
docker exec -it ur5yt bash

Next clone the repo of the 'UR5' branch 

Navigate to ur5_yt/

To build the separate packages, the following commands will need to be built with colcon and then sourced

colcon build
source /opt/ros/humble/setup.bash
source /workspaces/app_ws/install/setup.bash

Finally, run the first two commands for Gazebo launch and the last two commands to optionally print the running image detection and IK nodes
ros2 run gazebo_ros spawn_entity.py -entity cobot -topic robot_description
ros2 launch ur5_simulation spawn_ur5_camera_gripper_moveit.launch.py
ros2 run my_cv_package depth_detection
ros2 run ur5_manipulation run_ik
