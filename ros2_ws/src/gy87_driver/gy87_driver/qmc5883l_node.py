# qmc5883l_node.py
import time
from smbus2 import SMBus
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import MagneticField

QMC_ADDR      = 0x0D
REG_X_L       = 0x00  # X LSB, X MSB, Y LSB, Y MSB, Z LSB, Z MSB
REG_STATUS    = 0x06
REG_CONTROL_1 = 0x09
REG_CONTROL_2 = 0x0A
REG_SETRESET  = 0x0B

# Control1 bits: [OSR1 OSR0 RNG1 RNG0 ODR1 ODR0 MODE1 MODE0]
# We'll use: OSR=512 (00), RNG=2G (00), ODR=100Hz (10), MODE=Continuous (01) -> 0b00001001 = 0x09
CTRL1_CFG = 0x09
# Control2: soft reset bit 7
CTRL2_SOFT_RST = 0x80

# Scale: QMC5883L typical 2G range ~12000 LSB/Gauss -> 1 Gauss = 100 µT -> 100/12000 = 0.008333 µT / LSB
UT_PER_LSB = 100.0 / 12000.0  # ≈ 0.008333 µT/LSB

def read_axis(bus, addr):
    data = bus.read_i2c_block_data(addr, REG_X_L, 6)
    def to_i16(lo, hi):
        v = (hi << 8) | lo
        return v - 65536 if v & 0x8000 else v
    x = to_i16(data[0], data[1])
    y = to_i16(data[2], data[3])
    z = to_i16(data[4], data[5])
    return x, y, z

class QMC5883LNode(Node):
    def __init__(self):
        super().__init__('qmc5883l_node')
        self.declare_parameters('', [
            ('i2c_bus', 1),
            ('address', QMC_ADDR),
            ('frame_id', 'imu_link'),
            ('rate_hz', 50.0),
        ])
        self.bus_id = self.get_parameter('i2c_bus').value
        self.addr   = self.get_parameter('address').value
        self.frame  = self.get_parameter('frame_id').value
        self.rate   = float(self.get_parameter('rate_hz').value)

        self.bus = SMBus(self.bus_id)

        # Soft reset, set set/reset period, configure continuous mode
        try:
            self.bus.write_byte_data(self.addr, REG_CONTROL_2, CTRL2_SOFT_RST)
            time.sleep(0.01)
            self.bus.write_byte_data(self.addr, REG_SETRESET, 0x01)       # recommended
            self.bus.write_byte_data(self.addr, REG_CONTROL_1, CTRL1_CFG) # cont. 100Hz, 2G, OSR512
        except Exception as e:
            self.get_logger().error(f"QMC5883L init failed at 0x{self.addr:02X}: {e}")
            raise

        self.pub = self.create_publisher(MagneticField, 'imu/mag', 10)
        self.dt  = 1.0 / self.rate
        self.timer = self.create_timer(self.dt, self.loop)

        self.get_logger().info(f"QMC5883L on i2c-{self.bus_id} addr 0x{self.addr:02X} @ {self.rate} Hz")

    def loop(self):
        try:
            x, y, z = read_axis(self.bus, self.addr)
        except Exception as e:
            self.get_logger().warn(f"read failed: {e}")
            return

        # convert to Tesla
        Bx_uT = x * UT_PER_LSB
        By_uT = y * UT_PER_LSB
        Bz_uT = z * UT_PER_LSB

        msg = MagneticField()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.frame
        msg.magnetic_field.x = Bx_uT * 1e-6
        msg.magnetic_field.y = By_uT * 1e-6
        msg.magnetic_field.z = Bz_uT * 1e-6
        msg.magnetic_field_covariance = [1e-10,0,0, 0,1e-10,0, 0,0,1e-10]  # example
        self.pub.publish(msg)

def main():
    rclpy.init()
    node = QMC5883LNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
