import time

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge, CvBridgeError
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image
from std_msgs.msg import Int32MultiArray


class OrangeForwardNode(Node):
    def __init__(self) -> None:
        super().__init__('orange_forward_node')

        self.declare_parameter('image_topic', '/camera/camera/color/image_raw')
        self.declare_parameter('robot_command_topic', '/robot_command')
        self.declare_parameter('cooldown_sec', 2.0)
        self.declare_parameter('min_orange_fraction', 0.02)
        self.declare_parameter('min_blob_area', 5000.0)
        self.declare_parameter('debug_every_n_frames', 30)
        self.declare_parameter('orange_h_min', 5)
        self.declare_parameter('orange_h_max', 25)
        self.declare_parameter('orange_s_min', 120)
        self.declare_parameter('orange_v_min', 120)

        self.image_topic = self.get_parameter('image_topic').get_parameter_value().string_value
        self.robot_command_topic = (
            self.get_parameter('robot_command_topic').get_parameter_value().string_value
        )
        self.cooldown_sec = self.get_parameter('cooldown_sec').get_parameter_value().double_value
        self.min_orange_fraction = (
            self.get_parameter('min_orange_fraction').get_parameter_value().double_value
        )
        self.min_blob_area = self.get_parameter('min_blob_area').get_parameter_value().double_value
        self.debug_every_n_frames = (
            self.get_parameter('debug_every_n_frames').get_parameter_value().integer_value
        )

        self.lower_orange = np.array(
            [
                self.get_parameter('orange_h_min').get_parameter_value().integer_value,
                self.get_parameter('orange_s_min').get_parameter_value().integer_value,
                self.get_parameter('orange_v_min').get_parameter_value().integer_value,
            ],
            dtype=np.uint8,
        )
        self.upper_orange = np.array(
            [
                self.get_parameter('orange_h_max').get_parameter_value().integer_value,
                255,
                255,
            ],
            dtype=np.uint8,
        )

        self.bridge = CvBridge()
        self.frame_count = 0
        self.last_trigger_time = 0.0

        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )

        self.command_publisher = self.create_publisher(
            Int32MultiArray, self.robot_command_topic, 10
        )
        self.image_subscription = self.create_subscription(
            Image,
            self.image_topic,
            self.image_callback,
            qos,
        )

        self.get_logger().info(
            'Watching %s for orange objects. Publishing Int32MultiArray(data=[-20, -20]) on %s '
            'when detected.'
            % (self.image_topic, self.robot_command_topic)
        )
        self.get_logger().debug(
            'Orange thresholds: HSV lower=%s upper=%s, min_fraction=%.3f, min_blob_area=%.0f, '
            'cooldown=%.2fs, debug_every_n_frames=%d'
            % (
                self.lower_orange.tolist(),
                self.upper_orange.tolist(),
                self.min_orange_fraction,
                self.min_blob_area,
                self.cooldown_sec,
                self.debug_every_n_frames,
            )
        )

    def image_callback(self, msg: Image) -> None:
        self.frame_count += 1

        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except CvBridgeError as exc:
            self.get_logger().error(f'Failed to convert image: {exc}')
            return

        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, self.lower_orange, self.upper_orange)

        kernel = np.ones((5, 5), dtype=np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)

        orange_fraction = cv2.countNonZero(mask) / float(mask.size)
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        largest_blob_area = max((cv2.contourArea(contour) for contour in contours), default=0.0)

        if self.debug_every_n_frames > 0 and self.frame_count % self.debug_every_n_frames == 0:
            self.get_logger().debug(
                'Frame %d: orange_fraction=%.4f, largest_blob_area=%.0f, contour_count=%d'
                % (
                    self.frame_count,
                    orange_fraction,
                    largest_blob_area,
                    len(contours),
                )
            )

        if orange_fraction < self.min_orange_fraction or largest_blob_area < self.min_blob_area:
            return

        now = time.monotonic()
        if now - self.last_trigger_time < self.cooldown_sec:
            self.get_logger().debug(
                'Orange detected but cooldown active for %.2f more seconds.'
                % (self.cooldown_sec - (now - self.last_trigger_time))
            )
            return

        self.last_trigger_time = now

        command = Int32MultiArray()
        command.data = [-20, -20]
        self.command_publisher.publish(command)

        self.get_logger().info(
            'Orange detected (coverage=%.3f, largest_blob=%.0f). Commanding robot forward.'
            % (orange_fraction, largest_blob_area)
        )


def main(args=None) -> None:
    rclpy.init(args=args)
    node = OrangeForwardNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
