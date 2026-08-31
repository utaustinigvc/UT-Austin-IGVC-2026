import time

import rclpy
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Int32MultiArray


class LidarForwardNode(Node):
    def __init__(self) -> None:
        super().__init__('lidar_forward_node')

        self.declare_parameter('scan_topic', '/scan')
        self.declare_parameter('robot_command_topic', '/robot_command')
        self.declare_parameter('detection_range_m', 2.0)
        self.declare_parameter('forward_arc_deg', 30.0)
        self.declare_parameter('motor_speed', -20)
        self.declare_parameter('cooldown_sec', 2.0)

        self.scan_topic = self.get_parameter('scan_topic').get_parameter_value().string_value
        self.robot_command_topic = (
            self.get_parameter('robot_command_topic').get_parameter_value().string_value
        )
        self.detection_range_m = (
            self.get_parameter('detection_range_m').get_parameter_value().double_value
        )
        self.forward_arc_deg = (
            self.get_parameter('forward_arc_deg').get_parameter_value().double_value
        )
        self.motor_speed = self.get_parameter('motor_speed').get_parameter_value().integer_value
        self.cooldown_sec = self.get_parameter('cooldown_sec').get_parameter_value().double_value

        self.last_trigger_time = 0.0

        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )

        self.command_publisher = self.create_publisher(
            Int32MultiArray, self.robot_command_topic, 10
        )
        self.scan_subscription = self.create_subscription(
            LaserScan,
            self.scan_topic,
            self.scan_callback,
            qos,
        )

        self.get_logger().info(
            'Watching %s for objects within %.1fm in ±%.0f° forward arc. '
            'Publishing Int32MultiArray(data=[%d, %d]) on %s when detected.'
            % (
                self.scan_topic,
                self.detection_range_m,
                self.forward_arc_deg,
                self.motor_speed,
                self.motor_speed,
                self.robot_command_topic,
            )
        )

    def scan_callback(self, msg: LaserScan) -> None:
        import math

        arc_rad = math.radians(self.forward_arc_deg)

        # Find index range that covers the forward arc around angle 0
        num_ranges = len(msg.ranges)
        if num_ranges == 0:
            return

        angle_min = msg.angle_min
        angle_increment = msg.angle_increment

        detected = False
        for i, r in enumerate(msg.ranges):
            angle = angle_min + i * angle_increment
            # Normalize to [-pi, pi]
            while angle > math.pi:
                angle -= 2 * math.pi
            while angle < -math.pi:
                angle += 2 * math.pi

            if abs(angle) > arc_rad:
                continue
            if math.isnan(r) or math.isinf(r):
                continue
            if msg.range_min <= r <= self.detection_range_m:
                detected = True
                break

        if not detected:
            return

        now = time.monotonic()
        if now - self.last_trigger_time < self.cooldown_sec:
            return

        self.last_trigger_time = now

        command = Int32MultiArray()
        command.data = [self.motor_speed, self.motor_speed]
        self.command_publisher.publish(command)

        self.get_logger().info(
            'Object detected within %.1fm in forward arc. Commanding robot forward.'
            % self.detection_range_m
        )


def main(args=None) -> None:
    rclpy.init(args=args)
    node = LidarForwardNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
