import rclpy
import time
from rclpy.node import Node
from rclpy.action import ActionClient
from control_msgs.action import GripperCommand

from moveit_msgs.srv import GetPositionIK
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from control_msgs.action import FollowJointTrajectory


class UR5IKAndMove(Node):
    def __init__(self):
        super().__init__('ur5_ik_and_move')

        # IK service
        self.ik_client = self.create_client(GetPositionIK, '/compute_ik')
        while not self.ik_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for /compute_ik service...')

        # Change this to your actual controller action name
        self.traj_client = ActionClient(
            self,
            FollowJointTrajectory,
            '/joint_trajectory_controller/follow_joint_trajectory'
        )

     
        self.gripper_client = ActionClient(
            self,
            GripperCommand,
            '/gripper_position_controller/gripper_cmd'
        )
        

        self.current_joint_state = None
        self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_callback,
            10
        )

        self.arm_joint_names = [
            "shoulder_pan_joint",
            "shoulder_lift_joint",
            "elbow_joint",
            "wrist_1_joint",
            "wrist_2_joint",
            "wrist_3_joint"
        ]

    def joint_callback(self, msg):
        self.current_joint_state = msg

    def get_filtered_joint_state(self):
        if self.current_joint_state is None:
            return None

        js = self.current_joint_state
        filtered = JointState()
        filtered.name = self.arm_joint_names
        filtered.position = []

        for name in self.arm_joint_names:
            if name not in js.name:
                self.get_logger().error(f"Missing joint in /joint_states: {name}")
                return None
            idx = js.name.index(name)
            filtered.position.append(js.position[idx])

        return filtered
    def open_gripper(self):
        if not self.gripper_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error("Gripper action not available")
            return

        goal = GripperCommand.Goal()
        goal.command.position = 0.0   # open
        goal.command.max_effort = 50.0

        send_goal_future = self.gripper_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, send_goal_future)
    
    def close_gripper(self):
        if not self.gripper_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error("Gripper action not available")
            return

        goal = GripperCommand.Goal()
        goal.command.position = 0.35   # closed
        goal.command.max_effort = 2.0

        send_goal_future = self.gripper_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, send_goal_future)

    def compute_ik(self, x, y, z):
        while self.current_joint_state is None:
            rclpy.spin_once(self)

        req = GetPositionIK.Request()

        # Use your real MoveIt group name here
        req.ik_request.group_name = "ur5_manipulator"

        pose = PoseStamped()
        pose.header.frame_id = "base_link"
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.position.z = z
    
        pose.pose.orientation.x = 1.0
        pose.pose.orientation.y = 0.0
        pose.pose.orientation.z = 0.0
        pose.pose.orientation.w = 0.0
        #pose.pose.orientation.x = 0.707
        #pose.pose.orientation.y = 0.0
        #pose.pose.orientation.z = 0.707
        #pose.pose.orientation.w = 0.0
        
        req.ik_request.pose_stamped = pose

        seed = self.get_filtered_joint_state()
        if seed is None:
            return None

        req.ik_request.robot_state.joint_state = seed
        req.ik_request.timeout.sec = 2
        req.ik_request.avoid_collisions = True

        future = self.ik_client.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        res = future.result()

        if res is None:
            self.get_logger().error("IK service call failed")
            return None

        self.get_logger().info(f"IK error code: {res.error_code.val}")

        if res.error_code.val != 1:
            self.get_logger().error("IK failed")
            return None

        joint_map = dict(zip(res.solution.joint_state.name, res.solution.joint_state.position))

        target_positions = []
        for name in self.arm_joint_names:
            if name not in joint_map:
                self.get_logger().error(f"IK result missing joint: {name}")
                return None
            target_positions.append(joint_map[name])

        return target_positions

    def send_trajectory(self, target_positions):
        if not self.traj_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error("Trajectory action server not available")
            return

        current_js = self.get_filtered_joint_state()
        if current_js is None:
            return

        current_positions = current_js.position

        goal_msg = FollowJointTrajectory.Goal()
        goal_msg.trajectory = JointTrajectory()
        goal_msg.trajectory.joint_names = self.arm_joint_names

        # current point
        p1 = JointTrajectoryPoint()
        p1.positions = current_positions
        p1.time_from_start.sec = 0

        # target point
        p2 = JointTrajectoryPoint()
        p2.positions = target_positions
        p2.velocities = [0.0] * len(self.arm_joint_names)
        p2.time_from_start.sec = 6   # slower = stable

        goal_msg.trajectory.points = [p1, p2]

        self.get_logger().info("Sending trajectory...")

        send_goal_future = self.traj_client.send_goal_async(goal_msg)
        rclpy.spin_until_future_complete(self, send_goal_future)

        goal_handle = send_goal_future.result()
        if goal_handle is None or not goal_handle.accepted:
            self.get_logger().error("Trajectory rejected")
            return

        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)

        result = result_future.result()
        if result:
            self.get_logger().info(
                f"Result: {result.result.error_code} | {result.result.error_string}"
            )
       
    def run(self):
        target = self.compute_ik(0.538, -0.109, 0.289)
        print("\nIK target joint values:")
        for name, pos in zip(self.arm_joint_names, target):
            print(f"{name}: {pos*180/3.14:.3f}")

        self.open_gripper()
        # just move to target (no fake pre-grasp yet)
        pre = self.compute_ik(0.538, -0.109, 0.289+0.25)
        if pre is None:
            return
        self.send_trajectory(pre)
        time.sleep(5)

        # -------- DESCEND --------
        grasp = self.compute_ik(0.538, -0.109, 0.289+0.11)
        #0.02
        if grasp is None:
            return
        self.send_trajectory(grasp)
        self.close_gripper()
        time.sleep(5)

        # -------- LIFT --------
        lift = self.compute_ik(0.538, -0.109, 0.289+0.30)
        if lift is None:
            return
        self.send_trajectory(lift)
        time.sleep(5)


def main():
    rclpy.init()
    node = UR5IKAndMove()
    node.run()
    rclpy.shutdown()


if __name__ == '__main__':
    main()