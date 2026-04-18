#!/usr/bin/env python3
"""
================================================================================
  pick_place.py — Kinova Gen3 Lite Pick & Place Boilerplate
================================================================================

  WHAT THIS DOES:
    - Picks up all 5 differently-sized blocks one after another
    - Places each block at a unique location on the other side of the table
    - Automatically adjusts grasp height based on each block's size
    - Uses MoveIt2 for motion planning and GripperInterface for grasping

  HOW TO RUN:
    1. Make sure the robot/simulator and MoveIt are already running
    2. Spawn the blocks:
         bash spawn_blocks.sh
    3. Add the planning scene:
         ros2 run pick_place pick_place --ros-args -p task:=add_scene
    4. Run the task:
         ros2 run pick_place pick_place --ros-args -p task:=pick_place_all

  AVAILABLE TASKS (passed via -p task:=<n>):
    home            — move all joints to zero position
    retract         — move to a safe retracted pose
    add_scene       — add table and all 5 block collision objects to MoveIt
    pick_place_all  — pick and place all 5 blocks in succession

  GRASP HEIGHT CALCULATION:
    grasp_height = table_top + block_size/2 + GRASP_OFFSET
    GRASP_OFFSET accounts for the gripper not closing exactly at the block
    center. Tune this value if the gripper is consistently too high or low.

  TO EXTEND THIS:
    - Add new task methods and register them in the if/elif block in main()
    - Change PLACE_LOCATIONS to move the drop-off spots
================================================================================
"""

import time
import threading

import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from geometry_msgs.msg import Pose

from pymoveit2 import MoveIt2
from pymoveit2.gripper_interface import GripperInterface


# ==============================================================================
#  CONFIGURATION
# ==============================================================================

# Table top z in robot frame — must match spawn_blocks.sh TABLE_TOP_Z
TABLE_TOP_Z = -0.0001

# Offset added on top of block_size/2 when calculating grasp height.
# The gripper closes slightly above the true center of the block.
# Increase if gripper is too low, decrease if too high.
# Base grasp height that worked for a 0.05m block.
# For other sizes we adjust by half the size difference.
GRASP_HEIGHT_BASE = 0.185   # ← lowered slightly from 0.19 to help block_1
GRASP_SIZE_BASE   = 0.05

# Height to move to before descending to grasp — must clear all blocks
APPROACH_HEIGHT = 0.32

# End effector pointing straight down
TOP_DOWN_ORIENTATION = (-0.7071, 0.7071, 0.0, 0.0)

# Where to place each block after picking.
# One destination per block — in the same order as self.blocks.
# Format: (x, y)
PLACE_LOCATIONS = [
    (0.30,  0.20),   # place for block_1 (red,    0.03m)
    (0.30,  0.10),   # place for block_2 (blue,   0.04m)
    (0.30,  0.00),   # place for block_3 (green,  0.05m)
    (0.30, -0.10),   # place for block_4 (yellow, 0.06m)
    (0.30, -0.20),   # place for block_5 (orange, 0.07m)
]


# ==============================================================================
#  HELPER: make_pose
# ==============================================================================

def make_pose(x, y, z, qx, qy, qz, qw) -> Pose:
    """
    Build a geometry_msgs/Pose from position (x, y, z) and
    quaternion orientation (qx, qy, qz, qw).
    """
    p = Pose()
    p.position.x    = x
    p.position.y    = y
    p.position.z    = z
    p.orientation.x = qx
    p.orientation.y = qy
    p.orientation.z = qz
    p.orientation.w = qw
    return p


# ==============================================================================
#  MAIN NODE
# ==============================================================================

class PickAndPlaceNode(Node):

    def __init__(self):
        super().__init__("pick_and_place")

        # ----------------------------------------------------------------------
        #  ROS PARAMETER
        # ----------------------------------------------------------------------
        self.declare_parameter("task", "home")

        # ----------------------------------------------------------------------
        #  MOVEIT2
        # ----------------------------------------------------------------------
        self.moveit2 = MoveIt2(
            node=self,
            joint_names=["joint_1", "joint_2", "joint_3",
                         "joint_4", "joint_5", "joint_6"],
            base_link_name="base_link",
            end_effector_name="end_effector_link",
            group_name="arm",
        )

        # ----------------------------------------------------------------------
        #  GRIPPER INTERFACE
        #
        #  closed_gripper_joint_positions : tune per block if needed
        #    → smaller blocks may need a higher value to grip tightly
        # ----------------------------------------------------------------------
        try:
            self.gripper = GripperInterface(
                node=self,
                gripper_joint_names=["right_finger_bottom_joint"],
                open_gripper_joint_positions=[0.80],
                closed_gripper_joint_positions=[0.37],   # ← tune for grip strength
                gripper_group_name="gripper",
                gripper_command_action_name="/gen3_lite_2f_gripper_controller/gripper_cmd",
                ignore_new_calls_while_executing=False,  # ← must be False so open() works after close()
            )
        except Exception as e:
            self.get_logger().warn(f"GripperInterface initialization failed: {e}")
            self.gripper = None

        # ----------------------------------------------------------------------
        #  SCENE GEOMETRY — table
        # ----------------------------------------------------------------------
        self.table_center_z = TABLE_TOP_Z - 0.05 / 2.0

        # ----------------------------------------------------------------------
        #  BLOCK DEFINITIONS
        #
        #  Format: (x, y, size_m)
        #  Must match exactly what spawn_blocks.sh spawned in Gazebo.
        #
        #  block_1 — red,    0.03m, y=-0.16
        #  block_2 — blue,   0.04m, y=-0.08
        #  block_3 — green,  0.05m, y= 0.00
        #  block_4 — yellow, 0.06m, y= 0.08
        #  block_5 — orange, 0.07m, y= 0.16
        # ----------------------------------------------------------------------
        block_x = 0.45
        # Format: (x, y, size, close_position)
        # close_position: higher = tighter grip
        #   small blocks need tighter grip to actually grasp them
        #   large blocks need looser grip or fingers won't close around them
        self.blocks = [
            (block_x, -0.16, 0.04, 0.55),   # block_1 — red,    same size as block_2
            (block_x, -0.08, 0.04, 0.45),   # block_2 — blue
            (block_x,  0.00, 0.05, 0.37),   # block_3 — green
            (block_x,  0.08, 0.06, 0.30),   # block_4 — yellow
            (block_x,  0.16, 0.07, 0.25),   # block_5 — orange, looser grip
        ]

        # ----------------------------------------------------------------------
        #  JOINT CONFIGURATIONS
        # ----------------------------------------------------------------------
        self.j_home    = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        self.j_retract = [0.40, 0.02, 2.27, -1.57, -0.84, 1.97]

        # ----------------------------------------------------------------------
        #  TOUCH LINKS
        # ----------------------------------------------------------------------
        self.touch_links = [
            "end_effector_link",
            "right_finger_bottom_link",
            "left_finger_bottom_link",
        ]


    # ==========================================================================
    #  GRASP HEIGHT CALCULATOR
    # ==========================================================================

    def compute_grasp_height(self, block_size: float) -> float:
        """
        Automatically compute the grasp height for a given block size.

        Based on the known working grasp height of 0.19m for a 0.05m block.
        For smaller blocks we go slightly lower, for larger blocks slightly higher.

        Formula:
          grasp_height = GRASP_HEIGHT_BASE + (block_size - GRASP_SIZE_BASE) / 2.0
        """
        return GRASP_HEIGHT_BASE + (block_size - GRASP_SIZE_BASE) / 2.0


    # ==========================================================================
    #  MOTION WRAPPERS
    # ==========================================================================

    def move_to_joints(self, joint_positions):
        """Move the arm to a specific joint configuration."""
        self.moveit2.move_to_configuration(joint_positions=joint_positions)
        self.moveit2.wait_until_executed()

    def move_to_pose(self, pose: Pose, cartesian: bool = False) -> bool:
        """
        Plan and execute a move to a target Pose.

        cartesian=False : free joint-space planning (default)
        cartesian=True  : straight-line Cartesian path (use for up/down moves)

        Returns True if successful, False if planning failed.
        """
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
        """Open the gripper."""
        if not self.gripper:
            return
        self.gripper.open()

    def close_gripper(self):
        """Close the gripper."""
        if not self.gripper:
            return
        self.gripper.close()


    # ==========================================================================
    #  SCENE SETUP
    # ==========================================================================

    def add_scene(self):
        """
        Add the table and all 5 blocks to the MoveIt planning scene.
        Each block is added with its correct size and position.

        Run with: ros2 run pick_place pick_place --ros-args -p task:=add_scene
        """
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

        # All 5 blocks — each with its own size and computed center z
        for i, (x, y, size, _) in enumerate(self.blocks, start=1):
            block_center_z = TABLE_TOP_Z + size / 2.0
            self.moveit2.add_collision_box(
                id=f"block_{i}",
                size=(size, size, size),
                position=(x, y, block_center_z),
                quat_xyzw=(0, 0, 0, 1),
                frame_id="base_link",
            )
            time.sleep(0.05)
            self.get_logger().info(f"Added block_{i} size={size}m at y={y}")

        self.get_logger().info("Planning scene has been set up.")


    # ==========================================================================
    #  SINGLE BLOCK PICK AND PLACE
    # ==========================================================================

    def pick_and_place_block(self, block_index: int) -> bool:
        """
        Pick up one block and place it at its designated location.

        This is called in a loop by task_pick_place_all() for each block.

        Args:
            block_index: 0-based index into self.blocks and PLACE_LOCATIONS

        Returns:
            True if the full sequence succeeded, False if any step failed.
        """
        x, y, size, close_pos = self.blocks[block_index]
        place_x, place_y = PLACE_LOCATIONS[block_index]
        block_id = f"block_{block_index + 1}"

        # Compute grasp height automatically from block size
        grasp_height = self.compute_grasp_height(size)

        self.get_logger().info(
            f"--- Block {block_index + 1}/5 | size={size}m | "
            f"pick=({x},{y}) | place=({place_x},{place_y}) | "
            f"grasp_z={grasp_height:.4f} ---"
        )

        # Build poses for this block
        pre_grasp_pose = make_pose(x, y,       APPROACH_HEIGHT, *TOP_DOWN_ORIENTATION)
        grasp_pose     = make_pose(x, y,       grasp_height,    *TOP_DOWN_ORIENTATION)
        pre_place_pose = make_pose(place_x, place_y, APPROACH_HEIGHT, *TOP_DOWN_ORIENTATION)
        place_pose     = make_pose(place_x, place_y, grasp_height,    *TOP_DOWN_ORIENTATION)

        
        self.get_logger().info(f"Opening gripper for {block_id}...")
        self.open_gripper()
        time.sleep(1.0)
        self.open_gripper()  # call twice to guarantee fully open
        time.sleep(1.5)

        # -- PICK --
        self.get_logger().info(f"Approaching {block_id}...")
        if not self.move_to_pose(pre_grasp_pose): return False
        if not self.move_to_pose(grasp_pose, cartesian=True): return False

        
        self.get_logger().info(f"Grasping {block_id} with close_pos={close_pos}...")
        if self.gripper:
            self.gripper.closed_gripper_joint_positions = [close_pos]
        self.close_gripper()
        time.sleep(1.5)
        self.close_gripper()  # call twice to be sure
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
        if self.gripper:
            self.gripper.open_gripper_joint_positions = [0.80]
        self.open_gripper()
        time.sleep(0.5)

        self.moveit2.detach_collision_object(block_id)
        time.sleep(0.2)

        # Lift straight up away from placed block
        if not self.move_to_pose(pre_place_pose, cartesian=True): return False

        self.get_logger().info(f"{block_id} placed successfully.")
        return True


    # ==========================================================================
    #  TASK: PICK AND PLACE ALL 5 BLOCKS
    # ==========================================================================

    def task_pick_place_all(self):
        """
        Pick up all 5 blocks in order and place each at its designated location.

        Sequence:
          For each block (1 → 5):
            1. Open gripper
            2. Approach block from above
            3. Descend and grasp
            4. Lift up
            5. Move to place location
            6. Descend and release
            7. Lift up → go straight to next block
          After all blocks: return to retract pose
        """
        self.get_logger().info("Starting pick and place task for all 5 blocks.")
        time.sleep(3.0)  # wait for MoveIt services to be ready

        # Start from retract pose
        self.move_to_joints(self.j_retract)

        # Loop through all 5 blocks
        for i in range(len(self.blocks)):
            success = self.pick_and_place_block(i)
            if not success:
                self.get_logger().error(
                    f"Failed on block_{i+1}. Returning to retract and stopping."
                )
                self.move_to_joints(self.j_retract)
                return

        # All done — return to retract
        self.get_logger().info("All 5 blocks placed successfully!")
        self.move_to_joints(self.j_retract)


    # ==========================================================================
    #  SIMPLE UTILITY TASKS
    # ==========================================================================

    def task_home(self):
        """Move all joints to zero (straight up)."""
        self.move_to_joints(self.j_home)

    def task_retract(self):
        """Move to the safe retracted pose."""
        self.move_to_joints(self.j_retract)

    def task_add_scene(self):
        """Add collision objects to the MoveIt planning scene."""
        self.add_scene()


# ==============================================================================
#  ENTRY POINT
# ==============================================================================

def main(args=None):
    rclpy.init(args=args)
    node = PickAndPlaceNode()

    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(node)
    threading.Thread(target=executor.spin, daemon=True).start()

    task = node.get_parameter("task").get_parameter_value().string_value
    node.get_logger().info(f"Executing task: '{task}'")

    try:
        if   task == "home":              node.task_home()
        elif task == "retract":           node.task_retract()
        elif task == "add_scene":         node.task_add_scene()
        elif task == "pick_place_all":    node.task_pick_place_all()
        # ── Add new tasks here ──────────────────────────────────────────────
        # elif task == "pick_place_one":  node.task_pick_place_one()
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