#!/usr/bin/env python3


import time
import threading
import subprocess

import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from geometry_msgs.msg import Pose
from std_msgs.msg import Float64

from pymoveit2 import MoveIt2
from pymoveit2.gripper_interface import GripperInterface



#  CONFIGURATION
# Table top z in robot planning frame — must match spawn_blocks.sh TABLE_TOP_Z
TABLE_TOP_Z = -0.0001

# Table top z in world frame — used for block spawning
TABLE_TOP_Z_WORLD = 0.30

# Conveyor belt speed in m/s.
# Negative = belt surface moves in -Y direction (toward robot at y=0.0).
# Belt is oriented along Y — surface never moves toward the robot body.
BELT_SPEED = -0.1

# Distance from block spawn point (y=1.0) to pickup point (y=0.0) in meters.
BELT_TRAVEL_DISTANCE = 1.0

# Belt run time: stop belt slightly early so block doesn't overshoot y=0.0.
# At 0.1 m/s, exact travel = 10s. We run for 9s so block stops ~y=0.1.
BELT_TRAVEL_TIME = 12.5

# Fixed pickup point in the robot planning frame (all blocks arrive here).
PICKUP_X = 0.45
PICKUP_Y = 0.0

# Block spawn point in world frame (far Y end of belt).
SPAWN_X_WORLD = 0.45
SPAWN_Y_WORLD = 1.0

# Grasp height calibration
GRASP_HEIGHT_BASE = 0.185
GRASP_SIZE_BASE   = 0.05

# Height to move to before descending to grasp
APPROACH_HEIGHT = 0.32

# End effector pointing straight down
TOP_DOWN_ORIENTATION = (-0.7071, 0.7071, 0.0, 0.0)

# Where to place each block after picking (x, y) in robot planning frame.
PLACE_LOCATIONS = [
    (0.30,  0.20),   # place for block_1 (red,    0.04m)
    (0.30,  0.10),   # place for block_2 (blue,   0.04m)
    (0.30,  0.00),   # place for block_3 (green,  0.05m)
    (0.30, -0.10),   # place for block_4 (yellow, 0.06m)
    (0.30, -0.20),   # place for block_5 (orange, 0.07m)
]

import os
CONVEYOR_FILES_DIR = os.path.expanduser(
    "~/workspaces/pick-and-place-conveyor/ros2_ws/src/mems-toolkit/conveyor_files"
)

#  HELPER: make_pose

def make_pose(x, y, z, qx, qy, qz, qw) -> Pose:
    p = Pose()
    p.position.x    = x
    p.position.y    = y
    p.position.z    = z
    p.orientation.x = qx
    p.orientation.y = qy
    p.orientation.z = qz
    p.orientation.w = qw
    return p


#  MAIN NODE

class PickAndPlaceNode(Node):

    def __init__(self):
        super().__init__("pick_and_place")

        self.declare_parameter("task", "home")

        #  MOVEIT2
        self.moveit2 = MoveIt2(
            node=self,
            joint_names=["joint_1", "joint_2", "joint_3",
                         "joint_4", "joint_5", "joint_6"],
            base_link_name="base_link",
            end_effector_name="end_effector_link",
            group_name="arm",
        )

        #  GRIPPER
        try:
            self.gripper = GripperInterface(
                node=self,
                gripper_joint_names=["right_finger_bottom_joint"],
                open_gripper_joint_positions=[0.80],
                closed_gripper_joint_positions=[0.37],
                gripper_group_name="gripper",
                gripper_command_action_name="/gen3_lite_2f_gripper_controller/gripper_cmd",
                ignore_new_calls_while_executing=False,
            )
        except Exception as e:
            self.get_logger().warn(f"GripperInterface initialization failed: {e}")
            self.gripper = None

        #  CONVEYOR BELT PUBLISHER
        #  Publishes to /conveyor_belt/cmd_vel (bridged to gz via ros_gz_bridge)
        self._belt_pub = self.create_publisher(Float64, "/conveyor_belt/cmd_vel", 10)

        #  SCENE / BLOCK DEFINITIONS
        self.table_center_z = TABLE_TOP_Z - 0.05 / 2.0

        # Format: (size_m, close_position)
        # All blocks arrive at (PICKUP_X, PICKUP_Y) — only size varies.
        self.blocks = [
            (0.04, 0.55),   # block_1 — red
            (0.04, 0.45),   # block_2 — blue
            (0.05, 0.37),   # block_3 — green
            (0.06, 0.30),   # block_4 — yellow
            (0.07, 0.25),   # block_5 — orange
        ]

        # Block colors for spawning (r, g, b)
        self.block_colors = [
            (0.85, 0.10, 0.10),  # red
            (0.10, 0.10, 0.85),  # blue
            (0.10, 0.75, 0.10),  # green
            (0.90, 0.85, 0.10),  # yellow
            (0.80, 0.40, 0.00),  # orange
        ]

        #  JOINT CONFIGURATIONS
        self.j_home    = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        self.j_retract = [0.40, 0.02, 2.27, -1.57, -0.84, 1.97]

        #  TOUCH LINKS
        self.touch_links = [
            "end_effector_link",
            "right_finger_bottom_link",
            "left_finger_bottom_link",
        ]


    #  BELT CONTROL

    def belt_start(self):
        """Start the conveyor belt moving toward the robot."""
        msg = Float64()
        msg.data = BELT_SPEED
        self._belt_pub.publish(msg)
        self.get_logger().info(f"Belt started at {BELT_SPEED} m/s")

    def belt_stop(self):
        """Stop the conveyor belt."""
        msg = Float64()
        msg.data = 0.0
        self._belt_pub.publish(msg)
        self.get_logger().info("Belt stopped")

    #  BLOCK SPAWNING

    def spawn_block_on_belt(self, block_index: int):
        """
        Spawn a single block at the input end of the conveyor belt.
        Uses gz service directly so no dependency on spawn_blocks.sh process.
        """
        size, _ = self.blocks[block_index]
        r, g, b = self.block_colors[block_index]
        block_name = f"lab06_block_{block_index + 1}"

        z_world = TABLE_TOP_Z_WORLD + size / 2.0 + 0.01  # slight drop above belt

        # Write a temporary SDF for this block
        sdf_path = f"/tmp/conveyor_block_{block_index + 1}.sdf"
        mass = 0.02
        inertia = (mass * size**2) / 6.0

        sdf_content = f"""<?xml version="1.0" ?>
<sdf version="1.8">
  <model name="{block_name}">
    <static>false</static>
    <link name="link">
      <inertial>
        <mass>{mass}</mass>
        <inertia>
          <ixx>{inertia:.8e}</ixx>
          <iyy>{inertia:.8e}</iyy>
          <izz>{inertia:.8e}</izz>
          <ixy>0</ixy><ixz>0</ixz><iyz>0</iyz>
        </inertia>
      </inertial>
      <collision name="c">
        <geometry><box><size>{size} {size} {size}</size></box></geometry>
        <surface>
          <friction>
            <ode>
              <mu>2.0</mu>
              <mu2>2.0</mu2>
            </ode>
          </friction>
          <contact>
            <ode>
              <kp>1e5</kp>
              <kd>10.0</kd>
              <max_vel>0.10</max_vel>
              <min_depth>1e-5</min_depth>
            </ode>
          </contact>
        </surface>
      </collision>
      <visual name="v">
        <geometry><box><size>{size} {size} {size}</size></box></geometry>
        <material>
          <ambient>{r} {g} {b} 1</ambient>
          <diffuse>{r} {g} {b} 1</diffuse>
        </material>
      </visual>
    </link>
  </model>
</sdf>
"""
        with open(sdf_path, "w") as f:
            f.write(sdf_content)

        # Remove old instance if present
        self._gz_remove_model(block_name)
        time.sleep(0.2)

        # Spawn at belt input end
        payload = (
            f'sdf_filename: "{sdf_path}" '
            f'name: "{block_name}" '
            f'pose {{ position {{ x: {SPAWN_X_WORLD} y: {SPAWN_Y_WORLD} z: {z_world} }} }} '
            f'allow_renaming: false'
        )
        self._gz_spawn(payload)
        self.get_logger().info(
            f"Spawned {block_name} (size={size}m) at x={SPAWN_X_WORLD}, z={z_world:.3f}"
        )

    def _respawn_belt(self):
        """Remove and re-spawn the conveyor belt to reset the belt surface position."""
        belt_sdf = os.path.join(CONVEYOR_FILES_DIR, "conveyor_belt.sdf")
        self._gz_remove_model("conveyor_belt")
        time.sleep(0.5)
        payload = (
            f'sdf_filename: "{belt_sdf}" '
            f'name: "conveyor_belt" '
            f'pose {{ position {{ x: 0.45 y: 0.55 z: 0.29 }} }} '
            f'allow_renaming: false'
        )
        self._gz_spawn(payload)

    def _gz_remove_model(self, name: str):
        try:
            subprocess.run(
                ["gz", "service", "-s", "/world/empty/remove",
                 "--reqtype", "gz.msgs.Entity",
                 "--reptype", "gz.msgs.Boolean",
                 "--timeout", "2000",
                 "--req", f'name: "{name}" type: MODEL'],
                capture_output=True, timeout=3
            )
        except Exception:
            pass  # model may not exist yet

    def _gz_spawn(self, payload: str):
        try:
            subprocess.run(
                ["gz", "service", "-s", "/world/empty/create",
                 "--reqtype", "gz.msgs.EntityFactory",
                 "--reptype", "gz.msgs.Boolean",
                 "--timeout", "5000",
                 "--req", payload],
                capture_output=True, timeout=6
            )
        except Exception as e:
            self.get_logger().error(f"Failed to spawn block: {e}")


    #  GRASP HEIGHT

    def compute_grasp_height(self, block_size: float) -> float:
        return GRASP_HEIGHT_BASE + (block_size - GRASP_SIZE_BASE) / 2.0


    #  MOTION WRAPPERS

    def move_to_joints(self, joint_positions):
        self.moveit2.move_to_configuration(joint_positions=joint_positions)
        self.moveit2.wait_until_executed()

    def move_to_pose(self, pose: Pose, cartesian: bool = False) -> bool:
        traj = self.moveit2.plan(
            pose=pose,
            cartesian=cartesian,
            max_step=0.005,
            cartesian_fraction_threshold=0.90 if cartesian else None,
        )
        if traj is None:
            self.get_logger().error("Planning failed.")
            return False
        self.moveit2.execute(traj)
        self.moveit2.wait_until_executed()
        return True

    def open_gripper(self):
        if not self.gripper:
            return
        self.gripper.open()

    def close_gripper(self):
        if not self.gripper:
            return
        self.gripper.close()


    #  SCENE SETUP

    def add_scene(self):
        """Add table collision box to the MoveIt planning scene."""
        self.get_logger().info("Adding objects to the planning scene.")

        # Table
        self.moveit2.add_collision_box(
            id="table_top",
            size=(1.0, 1.0, 0.05),
            position=(0.0, 0.0, self.table_center_z),
            quat_xyzw=(0, 0, 0, 1),
            frame_id="base_link",
        )
        time.sleep(0.1)

        self.get_logger().info("Planning scene set up.")


    #  SINGLE BLOCK PICK AND PLACE (from conveyor pickup point)

    def pick_and_place_block(self, block_index: int) -> bool:
        """
        Pick one block from the fixed conveyor pickup point and place it.
        All blocks arrive at (PICKUP_X, PICKUP_Y) — only size and close_pos vary.
        """
        size, close_pos = self.blocks[block_index]
        place_x, place_y = PLACE_LOCATIONS[block_index]
        block_id = f"lab06_block_{block_index + 1}"

        grasp_height = self.compute_grasp_height(size)

        self.get_logger().info(
            f"--- Block {block_index + 1}/5 | size={size}m | "
            f"pick=({PICKUP_X},{PICKUP_Y}) | place=({place_x},{place_y}) | "
            f"grasp_z={grasp_height:.4f} ---"
        )

        pre_grasp_pose = make_pose(PICKUP_X, PICKUP_Y, APPROACH_HEIGHT, *TOP_DOWN_ORIENTATION)
        grasp_pose     = make_pose(PICKUP_X, PICKUP_Y, grasp_height,    *TOP_DOWN_ORIENTATION)
        pre_place_pose = make_pose(place_x,  place_y,  APPROACH_HEIGHT, *TOP_DOWN_ORIENTATION)
        place_pose     = make_pose(place_x,  place_y,  grasp_height,    *TOP_DOWN_ORIENTATION)

        # Open gripper before approaching
        self.get_logger().info(f"Opening gripper for {block_id}...")
        self.open_gripper()
        time.sleep(1.0)
        self.open_gripper()
        time.sleep(1.5)

        # -- PICK --
        self.get_logger().info(f"Approaching {block_id} at pickup point...")
        if not self.move_to_pose(pre_grasp_pose): return False
        if not self.move_to_pose(grasp_pose, cartesian=True): return False

        self.get_logger().info(f"Grasping {block_id} with close_pos={close_pos}...")
        if self.gripper:
            self.gripper.closed_gripper_joint_positions = [close_pos]
        self.close_gripper()
        time.sleep(1.5)
        self.close_gripper()
        time.sleep(0.5)

        self.moveit2.attach_collision_object(block_id, "end_effector_link", self.touch_links)
        time.sleep(0.2)

        # Lift straight up
        if not self.move_to_pose(pre_grasp_pose, cartesian=True): return False

        # -- PLACE --
        self.get_logger().info(f"Moving to place location for {block_id}...")
        if not self.move_to_pose(pre_place_pose): return False
        if not self.move_to_pose(place_pose, cartesian=True): return False

        self.get_logger().info(f"Releasing {block_id}...")
        if self.gripper:
            self.gripper.open_gripper_joint_positions = [0.80]
        self.open_gripper()
        time.sleep(2.0)
        self.open_gripper()
        time.sleep(0.5)

        self.moveit2.detach_collision_object(block_id)
        time.sleep(0.2)

        if not self.move_to_pose(pre_place_pose, cartesian=True): return False

        self.get_logger().info(f"{block_id} placed successfully.")
        return True


    #  TASK: PICK AND PLACE ALL 5 BLOCKS VIA CONVEYOR

    def task_pick_place_all(self):
        """
        Full conveyor pick-and-place sequence for all 5 blocks.

        For each block:
          1. Spawn block at belt input end (x=1.5)
          2. Start belt — block travels ~10s to pickup point (x=0.45)
          3. Stop belt
          4. Pick block from fixed pickup point
          5. Place block at its designated location
          6. Repeat for next block
        """
        self.get_logger().info("Starting conveyor pick-and-place for all 5 blocks.")
        time.sleep(3.0)  # wait for MoveIt services

        # Remove any blocks left over from a previous run
        self.get_logger().info("Cleaning up any existing blocks...")
        for i in range(len(self.blocks)):
            self._gz_remove_model(f"lab06_block_{i + 1}")
        time.sleep(0.5)

        self.move_to_joints(self.j_retract)

        for i in range(len(self.blocks)):
            self.get_logger().info(f"=== Block {i+1}/{len(self.blocks)}: resetting belt ===")

            # Respawn belt to reset surface position before each block
            self._respawn_belt()
            time.sleep(2.0)

            # Spawn block at belt input end
            self.get_logger().info(f"=== Block {i+1}/{len(self.blocks)}: spawning on belt ===")
            self.spawn_block_on_belt(i)
            time.sleep(0.5)

            # Run belt for BELT_TRAVEL_TIME seconds to deliver block to pickup point
            self.belt_start()
            time.sleep(BELT_TRAVEL_TIME)
            self.belt_stop()
            time.sleep(1.0)  # let block settle

            # Pick and place
            success = self.pick_and_place_block(i)
            if not success:
                self.get_logger().error(
                    f"Failed on block_{i+1}. Stopping belt and returning to retract."
                )
                self.belt_stop()
                self.move_to_joints(self.j_retract)
                return

        self.get_logger().info("All 5 blocks placed successfully!")
        self.move_to_joints(self.j_retract)


    #  UTILITY TASKS

    def task_home(self):
        self.move_to_joints(self.j_home)

    def task_retract(self):
        self.move_to_joints(self.j_retract)

    def task_add_scene(self):
        self.add_scene()


#  ENTRY POINT

def main(args=None):
    rclpy.init(args=args)
    node = PickAndPlaceNode()

    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(node)
    threading.Thread(target=executor.spin, daemon=True).start()

    task = node.get_parameter("task").get_parameter_value().string_value
    node.get_logger().info(f"Executing task: '{task}'")

    try:
        if   task == "home":            node.task_home()
        elif task == "retract":         node.task_retract()
        elif task == "add_scene":       node.task_add_scene()
        elif task == "pick_place_all":  node.task_pick_place_all()
        else:
            node.get_logger().warn(
                f"Unknown task '{task}'. "
                f"Valid tasks: home, retract, add_scene, pick_place_all"
            )
    finally:
        time.sleep(1)
        rclpy.shutdown()


if __name__ == "__main__":
    main()
