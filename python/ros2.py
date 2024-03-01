from mpu9250 import MPU9250
from madgwick import Madgwick
from orientation import quat2eul, eul2quat
import time
import numpy as np
import math

# ros2 dependency 
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu

# ENU:
# X (Pitch): clockwise - / anticlockwise +
# Y (Roll): clockwise - / anticlockwise +
# Z (Yaw): clockwise - / anticlockwise +

# NED:
# X (Roll): clockwise + / anticlockwise -
# Y (Pitch): clockwise + / anticlockwise -
# Z (Yaw): clockwise + / anticlockwise -

class ros2_node(Node):
    def __init__(self, name, hz):
        self.get_logger().info("ros2 node init")
        self.publisher = self.create_publisher(Imu, 'imu_data', 1)
        self.timer = self.create_timer(hz, self.timer_callback) # call the method according to hz

    def timer_callback(self):
        imu_data = Imu()
        imu_data.header.frame_id = "mpu9250"
        imu_data.header.stamp = self.get_clock().now().to_msg()
        imu_data.linear_acceleration.x = 0
        imu_data.linear_acceleration.y = 0
        imu_data.linear_acceleration.z = 0
        imu_data.angular_velocity.x = 0
        imu_data.angular_velocity.y = 0
        imu_data.angular_velocity.z = 0
        imu_data.orientation.x = 0
        imu_data.orientation.y = 0
        imu_data.orientation.z = 0
        imu_data.orientation.w = 0
        self.publisher.publish(imu_data)

def main(args=None):
    rclpy.init(args=args)
    imu_node = ros2_node("ros2_node", 100)
    rclpy.spin(imu_node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()
