The UR5e implementation uses a prebuilt Docker image running on ROS2 Humble Hawksbill and Gazebo classic. 

It allows you to run a UR5e robotic arm, perform inverse kinematics, and detect colored blocks on a wooden table. As of now, the arm has not yet had the gripper attachment feature working. 

### Installation
To start, run the container 
`docker run -it --name ur5yt --net=host -e DISPLAY=$DISPLAY -e QT_X11_NO_MITSHM=1 -e RMW_IMPLEMENTATION=rmw_cyclonedds_cpp -e ROS_DOMAIN_ID=0 -v /tmp/.X11-unix:/tmp/.X11-unix:rw -v /dev/dri:/dev/dri -v ~/workspaces/ur5_yt_docker/src:/workspaces/app_ws/src learnroboticswros/ur5_gripper_camera_sim:gui`

For each terminal you eventually need, you will need to type 
`xhost +si:localuser:root`

`docker exec -it ur5yt bash`

Next clone the repo of the 'UR5' branch 

Navigate to ur5_yt/

To build the separate packages, the following commands will need to be built with colcon and then sourced

`colcon build`

`source /opt/ros/humble/setup.bash`

`source /workspaces/app_ws/install/setup.bash`

Finally, run the first two commands for Gazebo launch and the last two commands to optionally print the running image detection and IK nodes
`ros2 run gazebo_ros spawn_entity.py -entity cobot -topic robot_description`

`ros2 launch ur5_simulation spawn_ur5_camera_gripper_moveit.launch.py`

`ros2 run my_cv_package depth_detection`

`ros2 run ur5_manipulation run_ik`

### Usage
The following are common use cases, 
To modify the world file, navigate to src/ ur5_simulation/worlds to inspect the current world file. It is recommended to visually modify the world in Gazebo after spawn and then save the files instead of manually writing xml. 

To modify the threshold R, G, B values, navigate to src/my_cv_package/my_cv_package/depth_detection.py

To modify the inverse kinematics, navigate to src/ur5_manipulation/ur5_manipulation/ik_node.py


### Issues
For OpenCV to work, you may need the following installations and downgrading numpy <2 such as 1.4
sudo apt-get update
sudo apt install ros-humble-cv-bridge
sudo apt-get install python3-opencv
pip install opencv-python

### Attributions
This project is based off significant contributions from the  docker image provided by LearnRoboticsWROS's work at https://github.com/LearnRoboticsWROS/ur5_yt_docker. 
