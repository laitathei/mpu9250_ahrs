#!/usr/bin/python3
from .mpu9250 import MPU9250
from .madgwick import Madgwick
from .orientation import quat2eul, eul2quat
import threading
import time
import numpy as np
import math

# ros2 dependency 
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu, MagneticField
from geometry_msgs.msg import Point

# ENU:
# X (Pitch): clockwise - / anticlockwise +
# Y (Roll): clockwise - / anticlockwise +
# Z (Yaw): clockwise - / anticlockwise +

# NED:
# X (Roll): clockwise + / anticlockwise -
# Y (Pitch): clockwise + / anticlockwise -
# Z (Yaw): clockwise + / anticlockwise -

class ros2_node(Node):
    def __init__(self, name, hz, frame, axis, gain):
        super().__init__(name)
        self.hz = hz
        self.frame = frame
        self.imu = MPU9250(nav_frame=self.frame)
        self.filter = Madgwick(axis, marg_gain=gain)
        imu_thread = threading.Thread(target=self.imu_threading)
        imu_thread.start()
        self.rpy_publisher = self.create_publisher(Point, 'rpy', 10)
        self.fixed_rpy_publisher = self.create_publisher(Point, 'fixed_rpy', 10)
        self.imu_publisher = self.create_publisher(Imu, 'imu', 10)
        self.mag_publisher = self.create_publisher(MagneticField, 'mag', 10)
        self.timer = self.create_timer(1/self.hz, self.timer_callback) # call the method according to hz
        self.get_logger().info("mpu9250 with madgwick filter ros2 node init")

    def imu_threading(self):
        count = 0
        interval = 1/self.hz
        last = time.time()
        self.ax, self.ay, self.az = self.imu.get_accel()
        self.gx, self.gy, self.gz = self.imu.get_gyro()
        self.mx, self.my, self.mz = self.imu.get_mag()
        self.roll, self.pitch, self.yaw = self.imu.get_rpy(axis=9)
        self.roll, self.pitch, self.yaw = round(self.roll,2), round(self.pitch,2), round(self.yaw,2)
        if self.frame == "NED":
            self.qw, self.qx, self.qy, self.qz = eul2quat(self.roll, self.pitch, self.yaw, seq="zxy")
        elif self.frame == "ENU":
            self.qw, self.qx, self.qy, self.qz = eul2quat(self.roll, self.pitch, self.yaw, seq="zyx")
        else:
            raise ValueError("Navigation frame should be either ENU or NED")
        self.filter.init_quat(self.qw, self.qx, self.qy, self.qz)
        try:
            while rclpy.ok():
                next = last + interval
                time.sleep(abs(next - time.time()))
                print("sleep time: ", abs(next - time.time()))
                self.ax, self.ay, self.az = self.imu.get_accel()
                self.gx, self.gy, self.gz = self.imu.get_gyro()
                self.mx, self.my, self.mz = self.imu.get_mag()
                self.ax, self.ay, self.az = round(self.ax,2), round(self.ay,2), round(self.az,2)
                self.gx, self.gy, self.gz = round(self.gx,2), round(self.gy,2), round(self.gz,2)
                self.mx, self.my, self.mz = round(self.mx,2), round(self.my,2), round(self.mz,2)
                self.roll, self.pitch, self.yaw = self.imu.get_rpy(axis=9)
                self.roll, self.pitch, self.yaw = round(self.roll,2), round(self.pitch,2), round(self.yaw,2)
                self.acc = np.array([[self.ax],[self.ay],[self.az]])
                self.gyr = np.array([[self.gx],[self.gy],[self.gz]])
                self.mag = np.array([[self.mx],[self.my],[self.mz]])
                self.fixed_qw, self.fixed_qx, self.fixed_qy, self.fixed_qz = self.filter.run(self.acc, self.gyr, self.mag, self.hz)
                if self.frame == "NED":
                    self.fixed_roll, self.fixed_pitch, self.fixed_yaw = quat2eul(self.fixed_qw, self.fixed_qx, self.fixed_qy, self.fixed_qz, seq="zyx")
                elif self.frame == "ENU":
                    self.fixed_roll, self.fixed_pitch, self.fixed_yaw = quat2eul(self.fixed_qw, self.fixed_qx, self.fixed_qy, self.fixed_qz, seq="zxy")
                else:
                    raise ValueError("Navigation frame should be either ENU or NED")
                self.fixed_roll, self.fixed_pitch, self.fixed_yaw = math.degrees(self.fixed_roll), math.degrees(self.fixed_pitch), math.degrees(self.fixed_yaw)
                self.fixed_roll, self.fixed_pitch, self.fixed_yaw = round(self.fixed_roll,2), round(self.fixed_pitch,2), round(self.fixed_yaw,2)
                last = time.time()
                count += 1
                print("")
                print(count)
                print("ax ay az: ", self.ax, self.ay, self.az)
                print("gx gy gz: ", self.gx, self.gy, self.gz)
                print("mx my mz: ", self.mx, self.my, self.mz)
                print("rpy: ", self.roll, self.pitch, self.yaw)
                print("fixed rpy: ", self.fixed_roll, self.fixed_pitch, self.fixed_yaw)
        
        except KeyboardInterrupt:
            print('interrupted!')

    def publish_euler_angle(self):
        point = Point()
        point.x = self.roll
        point.y = self.pitch
        point.z = self.yaw
        self.rpy_publisher.publish(point)

        fixed_point = Point()
        fixed_point.x = self.fixed_roll
        fixed_point.y = self.fixed_pitch
        fixed_point.z = self.fixed_yaw
        self.fixed_rpy_publisher.publish(fixed_point)

    def publish_imu_data(self):
        imu_data = Imu()
        imu_data.header.frame_id = "mpu9250"
        imu_data.header.stamp = self.get_clock().now().to_msg()
        imu_data.linear_acceleration.x = self.ax
        imu_data.linear_acceleration.y = self.ay
        imu_data.linear_acceleration.z = self.az
        imu_data.angular_velocity.x = self.gx
        imu_data.angular_velocity.y = self.gy
        imu_data.angular_velocity.z = self.gz
        imu_data.orientation.x = self.qx
        imu_data.orientation.y = self.qy
        imu_data.orientation.z = self.qz
        imu_data.orientation.w = self.qw
        self.imu_publisher.publish(imu_data)
        # self.get_logger().info("publish imu data")

    def publish_mag_data(self):
        mag_data = MagneticField()
        mag_data.header.frame_id = "mpu9250"
        mag_data.header.stamp = self.get_clock().now().to_msg()
        mag_data.magnetic_field.x = self.mx
        mag_data.magnetic_field.y = self.my
        mag_data.magnetic_field.z = self.mz
        self.mag_publisher.publish(mag_data)
        # self.get_logger().info("publish mag data")

    def timer_callback(self):
        self.publish_imu_data()
        self.publish_mag_data()
        self.publish_euler_angle()
    
def main(args=None):
    rclpy.init(args=args)
    imu_node = ros2_node(name="ros2_node", hz=100, frame="NED", axis=9, gain=5)
    rclpy.spin(imu_node)
    imu_node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
