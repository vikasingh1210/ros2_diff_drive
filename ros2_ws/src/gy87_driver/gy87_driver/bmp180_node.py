import time
from smbus2 import SMBus
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import FluidPressure, Temperature

BMP180_ADDR = 0x77

# BMP180 registers/constants (simplified)
REG_CALIB   = 0xAA
REG_CTRL_MEAS = 0xF4
REG_OUT_MSB = 0xF6
CMD_TEMP    = 0x2E
CMD_PRES    = 0x34  # + (oss<<6)
OSS         = 0     # oversampling 0..3

def read_i16s(bus, addr, reg, n_words):
    out = []
    for i in range(n_words):
        msb = bus.read_byte_data(addr, reg + 2*i)
        lsb = bus.read_byte_data(addr, reg + 2*i + 1)
        val = (msb << 8) | lsb
        if val & 0x8000: val -= 65536
        out.append(val)
    return out

class BMP180Node(Node):
    def __init__(self):
        super().__init__('bmp180_node')
        self.declare_parameters('', [
            ('i2c_bus', 1),
            ('address', BMP180_ADDR),
            ('frame_id', 'imu_link'),
            ('rate_hz', 10.0),
        ])
        self.bus_id = self.get_parameter('i2c_bus').value
        self.addr   = self.get_parameter('address').value
        self.frame  = self.get_parameter('frame_id').value
        self.rate   = float(self.get_parameter('rate_hz').value)

        self.bus = SMBus(self.bus_id)
        # read calibration
        ac1, ac2, ac3, ac4, ac5, ac6, b1, b2, mb, mc, md = read_i16s(self.bus, self.addr, REG_CALIB, 11)
        # save signed/unsigned as needed
        self.ac1, self.ac2, self.ac3 = ac1, ac2, ac3
        self.ac4, self.ac5, self.ac6 = (ac4 & 0xFFFF), (ac5 & 0xFFFF), (ac6 & 0xFFFF)
        self.b1, self.b2, self.mb, self.mc, self.md = b1, b2, mb, mc, md

        self.pub_p = self.create_publisher(FluidPressure, 'baro', 10)
        self.pub_t = self.create_publisher(Temperature, 'temperature', 10)
        self.dt = 1.0 / self.rate
        self.timer = self.create_timer(self.dt, self.loop)

        self.get_logger().info(f"BMP180 on i2c-{self.bus_id} addr 0x{self.addr:02X} @ {self.rate} Hz")

    def read_raw_temp(self):
        self.bus.write_byte_data(self.addr, REG_CTRL_MEAS, CMD_TEMP)
        time.sleep(0.005)
        msb = self.bus.read_byte_data(self.addr, REG_OUT_MSB)
        lsb = self.bus.read_byte_data(self.addr, REG_OUT_MSB+1)
        return (msb << 8) | lsb

    def read_raw_pres(self):
        self.bus.write_byte_data(self.addr, REG_CTRL_MEAS, CMD_PRES + (OSS<<6))
        time.sleep(0.005)
        msb = self.bus.read_byte_data(self.addr, REG_OUT_MSB)
        lsb = self.bus.read_byte_data(self.addr, REG_OUT_MSB+1)
        xlsb= self.bus.read_byte_data(self.addr, REG_OUT_MSB+2)
        up = ((msb << 16) + (lsb << 8) + xlsb) >> (8-OSS)
        return up

    def compensate(self, ut, up):
        # from datasheet
        x1 = ((ut - self.ac6) * self.ac5) >> 15
        x2 = (self.mc << 11) // (x1 + self.md)
        b5 = x1 + x2
        temp = (b5 + 8) >> 4  # 0.1 °C
        b6 = b5 - 4000
        x1 = (self.b2 * (b6 * b6 >> 12)) >> 11
        x2 = (self.ac2 * b6) >> 11
        x3 = x1 + x2
        b3 = (((self.ac1*4 + x3) << OSS) + 2) >> 2
        x1 = (self.ac3 * b6) >> 13
        x2 = (self.b1 * (b6*b6 >> 12)) >> 16
        x3 = ((x1 + x2) + 2) >> 2
        b4 = (self.ac4 * (x3 + 32768)) >> 15
        b7 = (up - b3) * (50000 >> OSS)
        if b7 < 0x80000000:
            p = (b7 * 2) // b4
        else:
            p = (b7 // b4) * 2
        x1 = (p >> 8) * (p >> 8)
        x1 = (x1 * 3038) >> 16
        x2 = (-7357 * p) >> 16
        p = p + ((x1 + x2 + 3791) >> 4)
        return temp/10.0, p  # °C, Pa

    def loop(self):
        ut = self.read_raw_temp()
        up = self.read_raw_pres()
        T, P = self.compensate(ut, up)

        msgp = FluidPressure()
        msgp.header.stamp = self.get_clock().now().to_msg()
        msgp.header.frame_id = self.frame
        msgp.fluid_pressure = float(P)         # Pascals
        msgp.variance = 12.0                   # example variance
        self.pub_p.publish(msgp)

        msgt = Temperature()
        msgt.header = msgp.header
        msgt.temperature = float(T)
        msgt.variance = 0.2
        self.pub_t.publish(msgt)

def main():
    rclpy.init()
    node = BMP180Node()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
