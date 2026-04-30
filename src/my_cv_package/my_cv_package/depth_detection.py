import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from visualization_msgs.msg import Marker
from cv_bridge import CvBridge
import cv2
import numpy as np

import tf2_ros
from tf_transformations import quaternion_matrix


class CubeDetector(Node):
    def __init__(self):
        super().__init__('cube_detector')

        self.marker_pub = self.create_publisher(Marker, '/cube_marker', 10)

        # Subscribers
        self.rgb_sub = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.rgb_callback,
            10
        )

        self.depth_sub = self.create_subscription(
            Image,
            '/camera/depth/image_raw',
            self.depth_callback,
            10
        )

        # Marker publisher
        self.marker_pub = self.create_publisher(Marker, '/cube_markers', 10)

        self.br = CvBridge()
        self.depth_image = None

        # TF (minimal setup)
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # -------- HSV ranges --------
        self.lower_red1 = np.array([0, 120, 70])
        self.upper_red1 = np.array([10, 255, 255])
        self.lower_red2 = np.array([170, 120, 70])
        self.upper_red2 = np.array([180, 255, 255])

        self.lower_green = np.array([40, 50, 50])
        self.upper_green = np.array([80, 255, 255])

       
        self.lower_blue = np.array([100, 40, 20])
        self.upper_blue = np.array([130, 255, 120])

        self.kernel = np.ones((5, 5), np.uint8)

        # -------- Camera intrinsics --------
        self.fx = 525.0
        self.fy = 525.0
        self.cx = 320.0
        self.cy = 240.0

    # ---------------- DEPTH ----------------
    def depth_callback(self, msg):
        self.depth_image = self.br.imgmsg_to_cv2(msg, desired_encoding='passthrough')

    # ---------------- COLOR MASK ----------------
    def detect_color(self, hsv, lower, upper):
        mask = cv2.inRange(hsv, lower, upper)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, self.kernel)
        return mask

    # ---------------- FIND CENTERS ----------------
    def get_centers(self, mask, frame, color_name, draw_color):
        centers = []
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        for c in contours:
            if cv2.contourArea(c) > 300:
                x, y, w, h = cv2.boundingRect(c)
                cx = x + w // 2
                cy = y + h // 2

                centers.append((cx, cy))

                cv2.rectangle(frame, (x, y), (x + w, y + h), draw_color, 2)
                cv2.putText(frame, color_name, (x, y - 5),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, draw_color, 2)

        return centers

    # ---------------- PIXEL → 3D ----------------
    def pixel_to_3d(self, u, v):
        if self.depth_image is None:
            return None

        if v >= self.depth_image.shape[0] or u >= self.depth_image.shape[1]:
            return None

        Z = self.depth_image[v, u]

        if Z == 0 or np.isnan(Z):
            return None

        X = (u - self.cx) * Z / self.fx
        Y = (v - self.cy) * Z / self.fy

        return np.array([X, Y, Z])

    # ---------------- CAMERA → BASE ----------------
    def camera_to_base(self, pt_camera):
        try:
            tf_msg = self.tf_buffer.lookup_transform(
                'base_link',
                'camera_optical_link',
                rclpy.time.Time()
            )

            t = tf_msg.transform.translation
            q = tf_msg.transform.rotation

            T = quaternion_matrix([q.x, q.y, q.z, q.w])
            T[0, 3] = t.x
            T[1, 3] = t.y
            T[2, 3] = t.z

            pt_base = T @ np.array([pt_camera[0], pt_camera[1], pt_camera[2], 1.0])
            return pt_base[:3]

        except Exception as e:
            self.get_logger().error(f"TF error: {e}")
            return None

    # ---------------- MARKER ----------------
    def publish_marker(self, pt, frame_id, color, marker_id):
        marker = Marker()

        marker.header.frame_id = frame_id
        marker.header.stamp = self.get_clock().now().to_msg()

        marker.ns = "cubes"
        marker.id = marker_id

        marker.type = Marker.SPHERE
        marker.action = Marker.ADD

        marker.pose.position.x = float(pt[0])
        marker.pose.position.y = float(pt[1])
        marker.pose.position.z = float(pt[2])

        marker.pose.orientation.w = 1.0

        marker.scale.x = 0.05
        marker.scale.y = 0.05
        marker.scale.z = 0.05

        marker.color.r = color[0]
        marker.color.g = color[1]
        marker.color.b = color[2]
        marker.color.a = 1.0

        self.marker_pub.publish(marker)

    def publish_cube_marker(self, x, y, z):
        marker = Marker()

        marker.header.frame_id = "base_link"
        marker.header.stamp = self.get_clock().now().to_msg()

        marker.ns = "cube"
        marker.id = 0

        marker.type = Marker.CUBE
        marker.action = Marker.ADD

        marker.pose.position.x = float(x)
        marker.pose.position.y = float(y)
        marker.pose.position.z = float(z)

        marker.pose.orientation.w = 1.0

        marker.scale.x = 0.05
        marker.scale.y = 0.05
        marker.scale.z = 0.05

        marker.color.r = 1.0
        marker.color.g = 0.0
        marker.color.b = 0.0
        marker.color.a = 1.0

        self.marker_pub.publish(marker)

    # ---------------- MAIN CALLBACK ----------------
    def rgb_callback(self, msg):
        frame = self.br.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        mask_red = self.detect_color(hsv, self.lower_red1, self.upper_red1) + \
                   self.detect_color(hsv, self.lower_red2, self.upper_red2)
        mask_green = self.detect_color(hsv, self.lower_green, self.upper_green)
        mask_blue = self.detect_color(hsv, self.lower_blue, self.upper_blue)

        red_pixels = self.get_centers(mask_red, frame, "red", (0, 0, 255))
        green_pixels = self.get_centers(mask_green, frame, "green", (0, 255, 0))
        blue_pixels = self.get_centers(mask_blue, frame, "blue", (255, 0, 0))

        marker_id = 0

        for color_name, pixels, color_rgb in [
            ("RED", red_pixels, (1.0, 0.0, 0.0)),
            ("GREEN", green_pixels, (0.0, 1.0, 0.0)),
            ("BLUE", blue_pixels, (0.0, 0.0, 1.0))
        ]:
            for (u, v) in pixels:
                pt_cam = self.pixel_to_3d(u, v)
                if pt_cam is None:
                    continue

                pt_base = self.camera_to_base(pt_cam)
                if pt_base is None:
                    continue

                self.get_logger().info(
                    f"[{color_name}] camera=({pt_cam[0]:.3f},{pt_cam[1]:.3f},{pt_cam[2]:.3f}) "
                    f"base=({pt_base[0]:.3f},{pt_base[1]:.3f},{pt_base[2]:.3f})"
                )

                # publish in base frame (important!)
                self.publish_marker(pt_base, "base_link", color_rgb, marker_id)
                self.publish_cube_marker(-0.193, -0.263, 1.352)
                marker_id += 1

        cv2.imshow("Cube Detection", frame)
        cv2.waitKey(1)

    

def main(args=None):
    rclpy.init(args=args)
    node = CubeDetector()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()