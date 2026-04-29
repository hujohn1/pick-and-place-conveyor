import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from moveit_msgs.action import MoveGroup
from rclpy.action import ActionClient


class TestPose(Node):
    def __init__(self):
        super().__init__('test_pose')

        self.client = ActionClient(self, MoveGroup, '/move_action')

        self.timer = self.create_timer(2.0, self.send_goal)

    def send_goal(self):
        self.timer.cancel()

        goal = MoveGroup.Goal()

        goal.request.group_name = "manipulator"
        goal.request.num_planning_attempts = 10
        goal.request.allowed_planning_time = 5.0

        pose = PoseStamped()
        pose.header.frame_id = "base_link"
        pose.pose.position.x = 0.4
        pose.pose.position.y = 0.0
        pose.pose.position.z = 0.6
        pose.pose.orientation.w = 1.0

        goal.request.goal_constraints.append(
            self.create_pose_constraint(pose)
        )

        self.get_logger().info("Sending goal...")
        self.client.send_goal_async(goal)

    def create_pose_constraint(self, pose):
        from moveit_msgs.msg import Constraints, PositionConstraint
        from shape_msgs.msg import SolidPrimitive

        pc = PositionConstraint()
        pc.header = pose.header
        pc.link_name = "tool0"

        box = SolidPrimitive()
        box.type = SolidPrimitive.SPHERE
        box.dimensions = [0.01]

        pc.constraint_region.primitives.append(box)
        pc.constraint_region.primitive_poses.append(pose.pose)
        pc.weight = 1.0

        c = Constraints()
        c.position_constraints.append(pc)
        return c


def main():
    rclpy.init()
    node = TestPose()
    rclpy.spin(node)


if __name__ == '__main__':
    main()