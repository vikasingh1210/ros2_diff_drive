import time
import numpy as np
from smbus2 import SMBus
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from .utils import apply_calibration

MPU6050_ADDR = 0x68
REG_PWR_MGMT_1 = 0x6B
REG_ACCEL_XOUT_H = 0x3B
REG_GYRO_XOUT_H  = 0x43
REG_ACCEL_CONFIG = 0x1C
REG_GYRO_CONFIG  = 0x1B

REG_INT_PIN_CFG = 0x37   # BYPASS_EN = bit1
REG_USER_CTRL   = 0x6A   # I2C_MST_EN = bit5


def read_i16(bus, addr, reg):
    hi = bus.read_byte_data(addr, reg)
    lo = bus.read_byte_data(addr, reg+1)
    val = (hi << 8) | lo
    return val - 65536 if val & 0x8000 else val

class MPU6050Node(Node):
    def __init__(self):
        super().__init__('mpu6050_node')
        self.declare_parameters('', [
            ('i2c_bus', 1),
            ('address', MPU6050_ADDR),
            ('frame_id', 'imu_link'),
            ('rate_hz', 100.0),
            # calibration: biases (g, rad/s) and scaling (unitless)
            ('accel_bias', [0.0, 0.0, 0.0]),
            ('accel_scale', [1.0, 1.0, 1.0]),
            ('gyro_bias',  [0.0, 0.0, 0.0]),
            ('gyro_scale', [1.0, 1.0, 1.0]),
            ('remove_gravity', True),
        ])
        self.bus_id = self.get_parameter('i2c_bus').value
        self.addr   = self.get_parameter('address').value
        self.frame  = self.get_parameter('frame_id').value
        self.rate   = float(self.get_parameter('rate_hz').value)
        self.accel_bias = self.get_parameter('accel_bias').value
        self.accel_scale= self.get_parameter('accel_scale').value
        self.gyro_bias  = self.get_parameter('gyro_bias').value
        self.gyro_scale = self.get_parameter('gyro_scale').value
        self.remove_g   = bool(self.get_parameter('remove_gravity').value)

        self.bus = SMBus(self.bus_id)

        # wake and set ranges: ±2g, ±250 dps
        self.bus.write_byte_data(self.addr, REG_PWR_MGMT_1, 0x00)
        self.bus.write_byte_data(self.addr, REG_ACCEL_CONFIG, 0x00)  # +/-2g
        self.bus.write_byte_data(self.addr, REG_GYRO_CONFIG,  0x00)  # +/-250dps
        time.sleep(0.05)

        # Disable MPU's internal I2C master, enable bypass so external mag is visible on bus
        uc = self.bus.read_byte_data(self.addr, REG_USER_CTRL)
        self.bus.write_byte_data(self.addr, REG_USER_CTRL, uc & (~0x20))   # clear I2C_MST_EN (bit5)

        ipc = self.bus.read_byte_data(self.addr, REG_INT_PIN_CFG)
        self.bus.write_byte_data(self.addr, REG_INT_PIN_CFG, ipc | 0x02)   # set BYPASS_EN (bit1)
        time.sleep(0.01)
        self.get_logger().info("MPU6050 I2C bypass enabled (mag should appear on bus)")


        self.pub = self.create_publisher(Imu, 'imu/data_raw', 50)
        self.dt = 1.0 / self.rate
        self.timer = self.create_timer(self.dt, self.loop)

        self.get_logger().info(f"MPU6050 on i2c-{self.bus_id} addr 0x{self.addr:02X} @ {self.rate} Hz")

    def loop(self):
        # read raw
        ax = read_i16(self.bus, self.addr, REG_ACCEL_XOUT_H)
        ay = read_i16(self.bus, self.addr, REG_ACCEL_XOUT_H+2)
        az = read_i16(self.bus, self.addr, REG_ACCEL_XOUT_H+4)
        gx = read_i16(self.bus, self.addr, REG_GYRO_XOUT_H)
        gy = read_i16(self.bus, self.addr, REG_GYRO_XOUT_H+2)
        gz = read_i16(self.bus, self.addr, REG_GYRO_XOUT_H+4)

        # scale to SI
        # accel: LSB/g = 16384 at ±2g → m/s^2
        ax_mps2 = (ax / 16384.0) * 9.80665
        ay_mps2 = (ay / 16384.0) * 9.80665
        az_mps2 = (az / 16384.0) * 9.80665
        # gyro: LSB/(°/s) = 131 at ±250 dps → rad/s
        gx_rps = (gx / 131.0) * (np.pi/180.0)
        gy_rps = (gy / 131.0) * (np.pi/180.0)
        gz_rps = (gz / 131.0) * (np.pi/180.0)

        acc = apply_calibration([ax_mps2, ay_mps2, az_mps2], self.accel_bias, self.accel_scale)
        gyr = apply_calibration([gx_rps, gy_rps, gz_rps], self.gyro_bias, self.gyro_scale)

        # optional: remove gravity (if your bias accounts for it, set remove_g=False)
        if self.remove_g:
            # naive: if robot is largely level, subtract +Z gravity estimate
            acc[2] -= 9.80665

        msg = Imu()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.frame

        # leave orientation empty (covariance[0] = -1 means "unknown")
        msg.orientation_covariance[0] = -1.0

        msg.linear_acceleration.x = float(acc[0])
        msg.linear_acceleration.y = float(acc[1])
        msg.linear_acceleration.z = float(acc[2])
        msg.linear_acceleration_covariance = [0.04,0,0, 0,0.04,0, 0,0,0.04]  # example 0.2^2

        msg.angular_velocity.x = float(gyr[0])
        msg.angular_velocity.y = float(gyr[1])
        msg.angular_velocity.z = float(gyr[2])
        msg.angular_velocity_covariance = [0.01,0,0, 0,0.01,0, 0,0,0.01]     # example 0.1^2

        self.pub.publish(msg)

def main():
    rclpy.init()
    node = MPU6050Node()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
