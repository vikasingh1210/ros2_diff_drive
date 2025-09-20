import time
import numpy as np
from smbus2 import SMBus
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import MagneticField
from .utils import apply_calibration

HMC5883L_ADDR = 0x1E
REG_CONFIG_A  = 0x00
REG_CONFIG_B  = 0x01
REG_MODE      = 0x02
REG_OUT_X_H   = 0x03

def read_i16(bus, addr, reg):
    hi = bus.read_byte_data(addr, reg)
    lo = bus.read_byte_data(addr, reg+1)
    val = (hi << 8) | lo
    return val - 65536 if val & 0x8000 else val

class HMC5883LNode(Node):
    def __init__(self):
        super().__init__('hmc5883l_node')
        self.declare_parameters('', [
            ('i2c_bus', 1),
            ('address', HMC5883L_ADDR),
            ('frame_id', 'imu_link'),
            ('rate_hz', 50.0),
            # calibration (hard/soft iron): bias (µT), scale matrix or per-axis scale
            ('mag_bias', [0.0, 0.0, 0.0]),
            ('mag_scale', [1.0, 1.0, 1.0]),
        ])
        self.bus_id = self.get_parameter('i2c_bus').value
        self.addr   = self.get_parameter('address').value
        self.frame  = self.get_parameter('frame_id').value
        self.rate   = float(self.get_parameter('rate_hz').value)
        self.mag_bias  = self.get_parameter('mag_bias').value
        self.mag_scale = self.get_parameter('mag_scale').value

        self.bus = SMBus(self.bus_id)
        # Config: 8-average, 15 Hz, normal; Gain ±1.3 Ga (1090 LSB/Gauss); continuous mode
        self.bus.write_byte_data(self.addr, REG_CONFIG_A, 0b01110000)  # 8 samples avg, 15 Hz
        self.bus.write_byte_data(self.addr, REG_CONFIG_B, 0b00100000)  # gain
        self.bus.write_byte_data(self.addr, REG_MODE,     0b00000000)  # continuous

        self.pub = self.create_publisher(MagneticField, 'imu/mag', 10)
        self.dt = 1.0 / self.rate
        self.timer = self.create_timer(self.dt, self.loop)
        self.get_logger().info(f"HMC5883L on i2c-{self.bus_id} addr 0x{self.addr:02X} @ {self.rate} Hz")

        # gain scale: 1090 LSB/Gauss => uT: 1 Gauss = 100 µT
        self.lsb_per_gauss = 1090.0
        self.ut_per_lsb = (100.0 / self.lsb_per_gauss)  # µT/LSB

    def loop(self):
        # order X,Z,Y in this chip’s register map
        x = read_i16(self.bus, self.addr, REG_OUT_X_H)
        z = read_i16(self.bus, self.addr, REG_OUT_X_H+4)
        y = read_i16(self.bus, self.addr, REG_OUT_X_H+2)

        # convert to microTesla
        Bx = x * self.ut_per_lsb
        By = y * self.ut_per_lsb
        Bz = z * self.ut_per_lsb

        # calibration
        B = apply_calibration([Bx, By, Bz], self.mag_bias, self.mag_scale)  # still in µT
        B_T = np.array(B) * 1e-6  # Tesla

        msg = MagneticField()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.frame
        msg.magnetic_field.x = float(B_T[0])
        msg.magnetic_field.y = float(B_T[1])
        msg.magnetic_field.z = float(B_T[2])
        msg.magnetic_field_covariance = [1e-10,0,0, 0,1e-10,0, 0,0,1e-10]  # example
        self.pub.publish(msg)

def main():
    rclpy.init()
    node = HMC5883LNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
