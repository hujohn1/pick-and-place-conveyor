import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np
import tf2_ros

class ImageSubscriber(Node):
    def __init__(self):
        super().__init__('image_subscriber')

        self.subscription = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.listener_callback,
            10
        )

        self.br = CvBridge()

        # HSV ranges
        self.lower_red1 = np.array([0, 120, 70])
        self.upper_red1 = np.array([10, 255, 255])
        self.lower_red2 = np.array([170, 120, 70])
        self.upper_red2 = np.array([180, 255, 255])

        self.lower_green = np.array([40, 50, 50])
        self.upper_green = np.array([80, 255, 255])

        self.lower_blue = np.array([90, 50, 50])
        self.upper_blue = np.array([130, 255, 255])

        self.kernel = np.ones((5, 5), np.uint8)

    def detect_color(self, hsv, lower, upper):
        mask = cv2.inRange(hsv, lower, upper)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, self.kernel)
        return mask

    def get_centers(self, mask, frame, color_name, draw_color):
        centers = []

        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        for c in contours:
            area = cv2.contourArea(c)

            if area > 300:  # filter noise
                x, y, w, h = cv2.boundingRect(c)
                cx = x + w // 2
                cy = y + h // 2

                centers.append((cx, cy))

                cv2.rectangle(frame, (x, y), (x + w, y + h), draw_color, 2)
                cv2.putText(frame, color_name, (x, y - 5),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, draw_color, 2)

        return centers

    def listener_callback(self, data):
        frame = self.br.imgmsg_to_cv2(data, desired_encoding='bgr8')

        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        # Masks
        mask_red = self.detect_color(hsv, self.lower_red1, self.upper_red1) + \
                   self.detect_color(hsv, self.lower_red2, self.upper_red2)

        mask_green = self.detect_color(hsv, self.lower_green, self.upper_green)
        mask_blue = self.detect_color(hsv, self.lower_blue, self.upper_blue)

        # Detect cubes
        red_centers = self.get_centers(mask_red, frame, "red", (0, 0, 255))
        green_centers = self.get_centers(mask_green, frame, "green", (0, 255, 0))
        blue_centers = self.get_centers(mask_blue, frame, "blue", (255, 0, 0))

        # Log results
        self.get_logger().info(
            f"Red: {red_centers} | Green: {green_centers} | Blue: {blue_centers}"
        )

        transform = tf_buffer.lookup_transform(
            'base_link',
            'camera_frame',
            rclpy.time.Time()
        )

        # Show result
        cv2.imshow("Cube Detection", frame)
        cv2.waitKey(1)


def main(args=None):
    rclpy.init(args=args)
    node = ImageSubscriber()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()