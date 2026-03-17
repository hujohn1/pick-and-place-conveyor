#!/usr/bin/env python3
"""
================================================================================
  pick_place.py — Kinova Gen3 Lite Pick & Place Boilerplate
================================================================================

  WHAT THIS DOES:
    - Picks up a single block from a known position on a table
    - Moves it to a different position on the same table
    - Uses MoveIt2 for motion planning and a GripperInterface for grasping

  HOW TO RUN:
    1. Make sure the robot/simulator and MoveIt are already running
    2. In one terminal, add the planning scene:
         ros2 run pick_place pick_place --ros-args -p task:=add_scene
    3. In another terminal, run the task:
         ros2 run pick_place pick_place --ros-args -p task:=pick_place_one

  AVAILABLE TASKS (passed via -p task:=<name>):
    home            — move all joints to zero position
    retract         — move to a safe retracted pose
    add_scene       — add table and block collision objects to MoveIt
    pick_place_one  — full pick and place sequence

  TO EXTEND THIS:
    - Add new poses in __init__() using make_pose()
    - Add new task methods like task_pick_place_two()
    - Register them in the if/elif block in main()
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
#  HELPER: make_pose
# ==============================================================================
def make_pose(x, y, z, qx, qy, qz, qw) -> Pose:
    """
    Build a geometry_msgs/Pose from position (x, y, z) and
    quaternion orientation (qx, qy, qz, qw).

    Use this everywhere you need to define a target pose for the robot.
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
        #  'task' is passed on the command line via --ros-args -p task:=<name>
        #  Defaults to "home" if not provided.
        # ----------------------------------------------------------------------
        self.declare_parameter("task", "home")

        # ----------------------------------------------------------------------
        #  MOVEIT2 — motion planning interface
        #
        #  joint_names       : must match your URDF joint names exactly
        #  base_link_name    : root frame of the robot
        #  end_effector_name : the link MoveIt plans to move to a target pose
        #  group_name        : planning group name from the MoveIt config
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
        #  open_gripper_joint_positions   : joint value when fully open
        #  closed_gripper_joint_positions : joint value when grasping
        #    → increase this value to grip tighter (0.01 = barely closed,
        #      0.80 = nearly fully closed)
        #  gripper_command_action_name    : ROS action server for the gripper
        # ----------------------------------------------------------------------
        try:
            self.gripper = GripperInterface(
                node=self,
                gripper_joint_names=["right_finger_bottom_joint"],
                open_gripper_joint_positions=[0.80],
                closed_gripper_joint_positions=[0.37],   # ← tune for grip strength
                gripper_group_name="gripper",
                gripper_command_action_name="/gen3_lite_2f_gripper_controller/gripper_cmd",
                ignore_new_calls_while_executing=True,
            )
        except Exception as e:
            self.get_logger().warn(f"GripperInterface initialization failed: {e}")
            self.gripper = None  # gripper calls will be silently skipped

        # ----------------------------------------------------------------------
        #  SCENE GEOMETRY
        #
        #  These values must match what was spawned in Gazebo by your
        #  spawn script (spawn_blocks.sh).
        #
        #  table_z        : z-position of the top surface of the table
        #  block_size     : side length of the cube (meters)
        #  block_center_z : z at the center of the block
        #                   (table surface + half the block height)
        # ----------------------------------------------------------------------
        table_z              = -0.0001          # top of table in robot frame
        block_size           = 0.04             # 4 cm cube
        self.table_center_z  = table_z - 0.05 / 2.0        # center of table collision box
        self.block_center_z  = table_z + block_size / 2.0  # center of block

        block_x = 0.45      # how far forward the block is (meters)

        # List of block positions — add more tuples here for more blocks
        # Format: (x, y, z)
        self.blocks_xyz = [
            (block_x, 0.00, self.block_center_z),   # block_1 — the red block
            # (block_x, 0.08, self.block_center_z), # block_2 — uncomment to add more
        ]

        # ----------------------------------------------------------------------
        #  MOTION PARAMETERS
        #
        #  approach_height     : z-height to move to BEFORE descending to grasp
        #                        (should clear the block with room to spare)
        #  grasp_height        : z-height at which the gripper closes on the block
        #                        (tune this if the gripper is too high or too low)
        #  top_down_orientation: quaternion for a straight-down end effector
        #                        (-0.7071, 0.7071, 0, 0) = pointing straight down
        # ----------------------------------------------------------------------
        self.approach_height      = 0.32
        self.grasp_height         = 0.19    # ← tune if gripper misses the block
        self.top_down_orientation = (-0.7071, 0.7071, 0.0, 0.0)

        # ----------------------------------------------------------------------
        #  PICK POSES — where to pick up the block
        #
        #  pre_grasp_pose : hover above the block before descending
        #  grasp_pose     : descend to this height to close the gripper
        # ----------------------------------------------------------------------
        pick_xyz = self.blocks_xyz[0]   # picking block_1
        self.pre_grasp_pose = make_pose(
            pick_xyz[0], pick_xyz[1], self.approach_height, *self.top_down_orientation
        )
        self.grasp_pose = make_pose(
            pick_xyz[0], pick_xyz[1], self.grasp_height, *self.top_down_orientation
        )

        # ----------------------------------------------------------------------
        #  PLACE POSES — where to put the block down
        #
        #  Change place_xyz to move the block to a different spot on the table.
        #  Keep x ~0.45 and y within ±0.20 to stay on the table.
        # ----------------------------------------------------------------------
        place_xyz = (block_x, 0.15, self.block_center_z)   # ← change this to move elsewhere
        self.pre_place_pose = make_pose(
            place_xyz[0], place_xyz[1], self.approach_height, *self.top_down_orientation
        )
        self.place_pose = make_pose(
            place_xyz[0], place_xyz[1], self.grasp_height, *self.top_down_orientation
        )

        # ----------------------------------------------------------------------
        #  JOINT CONFIGURATIONS
        #
        #  j_home    : all joints at zero — straight up position
        #  j_retract : a safe tucked-back pose to start/end tasks from
        #              (avoids collisions when the arm is idle)
        # ----------------------------------------------------------------------
        self.j_home    = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        self.j_retract = [0.40, 0.02, 2.27, -1.57, -0.84, 1.97]

        # ----------------------------------------------------------------------
        #  TOUCH LINKS
        #
        #  Links that are allowed to touch the grasped object.
        #  MoveIt will not treat contact between these links and block_1
        #  as a collision after attach_collision_object() is called.
        # ----------------------------------------------------------------------
        self.touch_links = [
            "end_effector_link",
            "right_finger_bottom_link",
            "left_finger_bottom_link",
        ]


    # ==========================================================================
    #  MOTION WRAPPERS
    #  Clean wrappers around MoveIt2 and GripperInterface calls.
    #  Use these in your task methods instead of calling MoveIt2 directly.
    # ==========================================================================

    def move_to_joints(self, joint_positions):
        """Move the arm to a specific joint configuration."""
        self.moveit2.move_to_configuration(joint_positions=joint_positions)
        self.moveit2.wait_until_executed()

    def move_to_pose(self, pose: Pose, cartesian: bool = False) -> bool:
        """
        Plan and execute a move to a target Pose.

        cartesian=False : MoveIt plans freely through joint space (default)
        cartesian=True  : forces a straight-line Cartesian path
                          (use this for controlled up/down motions near objects)

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
        """Open the gripper. Silently skipped if gripper failed to initialize."""
        if not self.gripper:
            return
        self.gripper.open()

    def close_gripper(self):
        """Close the gripper to grasp an object."""
        if not self.gripper:
            return
        self.gripper.close()


    # ==========================================================================
    #  SCENE SETUP
    # ==========================================================================

    def add_scene(self):
        """
        Add collision objects to the MoveIt planning scene.

        Run this BEFORE pick_place_one so MoveIt knows about the table
        and blocks and can plan around them without collisions.

        Run with: ros2 run pick_place pick_place --ros-args -p task:=add_scene
        """
        self.get_logger().info("Adding objects to the planning scene.")

        # Add the table surface as a flat collision box
        self.moveit2.add_collision_box(
            id="table_top",
            size=(1.0, 1.0, 0.05),
            position=(0.0, 0.0, self.table_center_z),
            quat_xyzw=(0, 0, 0, 1),
            frame_id="base_link",
        )
        time.sleep(0.1)

        # Add each block as a small collision box
        # IDs are automatically: block_1, block_2, etc.
        for i, (x, y, z) in enumerate(self.blocks_xyz, start=1):
            self.moveit2.add_collision_box(
                id=f"block_{i}",
                size=(0.04, 0.04, 0.04),
                position=(x, y, z),
                quat_xyzw=(0, 0, 0, 1),
                frame_id="base_link",
            )
            time.sleep(0.05)

        self.get_logger().info("Planning scene has been set up.")


    # ==========================================================================
    #  TASK: PICK AND PLACE ONE BLOCK
    # ==========================================================================

    def task_pick_place_one(self):
        """
        Full pick and place sequence for block_1 (the red block).

        Sequence:
          1. Move to retract pose and open gripper
          2. Move above the block           (pre_grasp_pose, joint space)
          3. Descend to grasp height        (grasp_pose, cartesian)
          4. Close gripper + attach block in MoveIt
          5. Lift straight up               (pre_grasp_pose, cartesian)
          6. Move above place location      (pre_place_pose, joint space)
          7. Descend to place height        (place_pose, cartesian)
          8. Open gripper + detach block in MoveIt
          9. Lift straight up and retract
        """
        self.get_logger().info("Starting pick and place task for 'block_1'.")
        time.sleep(3.0)  # wait for MoveIt services to be fully ready

        # ------------------------------------------------------------------
        # 1. INITIAL SETUP
        # ------------------------------------------------------------------
        self.move_to_joints(self.j_retract)
        self.open_gripper()
        time.sleep(1.0)  # give gripper time to fully open

        # ------------------------------------------------------------------
        # 2-3. APPROACH BLOCK
        # ------------------------------------------------------------------
        self.get_logger().info("--- PICK SEQUENCE ---")

        # Move above the block (free joint-space planning)
        if not self.move_to_pose(self.pre_grasp_pose): return

        # Descend straight down to grasp height (cartesian = straight line)
        if not self.move_to_pose(self.grasp_pose, cartesian=True): return

        # ------------------------------------------------------------------
        # 4. GRASP
        # ------------------------------------------------------------------
        self.get_logger().info("Closing gripper to grasp block...")
        self.close_gripper()
        time.sleep(1.0)  # let contacts settle in simulation

        # Tell MoveIt the block is now attached to the end effector
        # so it won't plan through it
        self.moveit2.attach_collision_object("block_1", "end_effector_link", self.touch_links)
        time.sleep(0.2)

        # ------------------------------------------------------------------
        # 5. LIFT
        # ------------------------------------------------------------------
        # Move straight up back to approach height (cartesian = straight line)
        if not self.move_to_pose(self.pre_grasp_pose, cartesian=True): return

        # ------------------------------------------------------------------
        # 6-7. MOVE TO PLACE LOCATION
        # ------------------------------------------------------------------
        self.get_logger().info("--- PLACE SEQUENCE ---")

        # Move above the place location (free joint-space planning)
        if not self.move_to_pose(self.pre_place_pose): return

        # Descend straight down to place height (cartesian = straight line)
        if not self.move_to_pose(self.place_pose, cartesian=True): return

        # ------------------------------------------------------------------
        # 8. RELEASE
        # ------------------------------------------------------------------
        self.open_gripper()
        time.sleep(1.0)  # let the block settle before detaching

        # Tell MoveIt the block is no longer attached
        self.moveit2.detach_collision_object("block_1")
        time.sleep(0.2)

        # ------------------------------------------------------------------
        # 9. RETURN HOME
        # ------------------------------------------------------------------
        # Lift straight up away from the placed block
        self.move_to_pose(self.pre_place_pose, cartesian=True)

        # Return to safe retracted pose
        self.move_to_joints(self.j_retract)

        self.get_logger().info("Pick and place task completed successfully.")


    # ==========================================================================
    #  SIMPLE UTILITY TASKS
    # ==========================================================================

    def task_home(self):
        """Move all joints to zero (straight up). Good for checking robot state."""
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

    # MoveIt2 requires a multi-threaded executor to handle callbacks
    # while the main thread blocks on move/wait calls
    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(node)
    threading.Thread(target=executor.spin, daemon=True).start()

    # Read the task parameter and dispatch to the correct method
    task = node.get_parameter("task").get_parameter_value().string_value
    node.get_logger().info(f"Executing task: '{task}'")

    try:
        if   task == "home":            node.task_home()
        elif task == "retract":         node.task_retract()
        elif task == "add_scene":       node.task_add_scene()
        elif task == "pick_place_one":  node.task_pick_place_one()
        # ── Add new tasks here ──────────────────────────────────────────────
        # elif task == "pick_place_two":  node.task_pick_place_two()
        else:
            node.get_logger().warn(
                f"Unknown task '{task}'. "
                f"Valid tasks: home, retract, add_scene, pick_place_one"
            )
    finally:
        time.sleep(1)
        rclpy.shutdown()


if __name__ == "__main__":
    main()