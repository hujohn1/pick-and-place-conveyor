
 ROS 2 and Gazebo Harmonic simulation of an automated conveyor belt pick-and-place system using the Kinova Gen3 Lite 6-DOF robot arm. Five blocks of varying sizes are delivered one at a time via a custom conveyor belt, picked up by the robot, and sorted to designated locations using MoveIt 2 for motion planning.


How to Run

Open five terminals and run each command in order, waiting for each to finish loading before starting the next.

Terminal 1 — Gazebo + Robot**
```bash
ros2 launch kortex_bringup kortex_sim_control.launch.py sim_gazebo:=true robot_type:=gen3_lite gripper:=gen3_lite_2f robot_name:=gen3_lite dof:=6 use_sim_time:=true launch_rviz:=false robot_controller:=joint_trajectory_controller
```

Terminal 2 — MoveIt**
```bash
ros2 launch kinova_gen3_lite_moveit_config sim.launch.py use_sim_time:=true
```

Terminal 3 — ROS-Gz Bridge**
```bash
bash ~/workspaces/pick-and-place-conveyor/ros2_ws/src/mems-toolkit/conveyor_files/start_bridge.sh
```

Terminal 4 — Add Planning Scene (run once per session)**
```bash
source ~/workspaces/pick-and-place-conveyor/ros2_ws/install/setup.bash && ros2 run pick_place pick_place --ros-args -p task:=add_scene
```

Terminal 5 — Build and Run**
```bash
cd ~/workspaces/pick-and-place-conveyor/ros2_ws && colcon build --packages-select pick_place && source install/setup.bash && ros2 run pick_place pick_place --ros-args -p task:=pick_place_all
```

For consecutive runs in the same session, Terminal 5 can be shortened to:
```bash
ros2 run pick_place pick_place --ros-args -p task:=pick_place_all
```
