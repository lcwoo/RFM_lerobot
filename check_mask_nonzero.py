#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
import numpy as np

TOPIC = "/perception/target_object_mask"

class Check(Node):
    def __init__(self):
        super().__init__("check_mask_nonzero")
        self.sub = self.create_subscription(Image, TOPIC, self.cb, 10)

    def cb(self, msg: Image):
        if msg.encoding.lower() != "32sc1":
            self.get_logger().info(f"encoding={msg.encoding} (expected 32SC1)")
            return
        m = np.frombuffer(msg.data, dtype=np.int32).reshape(msg.height, msg.width)
        nz = int(np.count_nonzero(m))
        mx = int(m.max()) if m.size else 0
        self.get_logger().info(f"mask nonzero={nz}, max={mx}, shape=({msg.height},{msg.width})")

def main():
    rclpy.init()
    n = Check()
    rclpy.spin(n)

if __name__ == "__main__":
    main()
