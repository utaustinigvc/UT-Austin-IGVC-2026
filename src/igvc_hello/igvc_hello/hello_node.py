

#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
class Hello(Node):
    def __init__(self):
        super().__init__('hello')
        self.pub = self.create_publisher(String, 'chatter', 10)
        self.timer = self.create_timer(0.5, self.tick)
        self.i = 0
    def tick(self):
        msg = String()
        msg.data = f"hello IGVC {self.i}"
        self.pub.publish(msg)
        self.i += 1
def main():
    rclpy.init()
    rclpy.spin(Hello())
    rclpy.shutdown()
if __name__ == '__main__':
    main()
