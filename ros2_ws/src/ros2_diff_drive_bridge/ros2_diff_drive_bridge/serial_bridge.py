import threading, time, math
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32, Int16, Int32
from sensor_msgs.msg import JointState
import serial

def clamp(v, lo, hi): return lo if v < lo else (hi if v > hi else v)

class ArduinoBridge(Node):
    def __init__(self):
        super().__init__("arduino_diffdrive_bridge")
        self.declare_parameters("", [
            ("port", "/dev/ttyACM0"), ("baud", 115200),
            ("wheel_radius", 0.0325), ("wheel_track", 0.19),
            ("control_hz", 20.0), ("rpm_limit", 250.0), ("timeout_sec", 0.6),
            ("kp", 1.1), ("ki", 0.25),
            ("trim_left", 1.0), ("trim_right", 1.0),
            ("ff_k", 0.0),
        ])
        p = self.get_parameter
        self.port = p("port").value; self.baud = int(p("baud").value)
        self.r = float(p("wheel_radius").value); self.track = float(p("wheel_track").value)
        self.control_hz = float(p("control_hz").value); self.rpm_limit = float(p("rpm_limit").value)
        self.timeout_sec = float(p("timeout_sec").value)
        self.kp = float(p("kp").value); self.ki = float(p("ki").value)
        self.trim_left = float(p("trim_left").value); self.trim_right = float(p("trim_right").value)
        self.ff_k = float(p("ff_k").value)

        # shared serial handle + guards
        self.ser = None
        self.ser_lock = threading.Lock()
        self.stop_event = threading.Event()

        self.target_rpm_L = 0.0; self.target_rpm_R = 0.0
        self.last_cmd_time = self.get_clock().now()

        qos = QoSProfile(reliability=QoSReliabilityPolicy.BEST_EFFORT,
                         history=QoSHistoryPolicy.KEEP_LAST, depth=10)
        self.create_subscription(Twist, "cmd_vel", self.on_cmd_vel, qos)
        self.pub_rpm_L = self.create_publisher(Float32, "left_rpm", 10)
        self.pub_rpm_R = self.create_publisher(Float32, "right_rpm", 10)
        self.pub_pwm_L = self.create_publisher(Int16,  "left_pwm", 10)
        self.pub_pwm_R = self.create_publisher(Int16,  "right_pwm", 10)
        self.pub_tck_L = self.create_publisher(Int32,  "left_ticks", 10)
        self.pub_tck_R = self.create_publisher(Int32,  "right_ticks", 10)
        self.pub_js    = self.create_publisher(JointState, "wheel_joint_states", 10)

        # send SPD at fixed rate (runs in ROS executor thread)
        self.timer = self.create_timer(1.0 / self.control_hz, self.control_tick)

        # single IO worker that opens -> reads -> on error closes and retries
        self.io_thread = threading.Thread(target=self._io_worker, daemon=True)
        self.io_thread.start()

    # ---------- ROS I/O ----------
    def on_cmd_vel(self, msg: Twist):
        v = float(msg.linear.x); w = float(msg.angular.z)
        vL = v - 0.5 * w * self.track; vR = v + 0.5 * w * self.track
        factor = 60.0 / (2.0 * math.pi * self.r)  # m/s -> RPM
        self.target_rpm_L = clamp(factor * vL, -self.rpm_limit, self.rpm_limit)
        self.target_rpm_R = clamp(factor * vR, -self.rpm_limit, self.rpm_limit)
        self.last_cmd_time = self.get_clock().now()

    def control_tick(self):
        # watchdog: if no cmd_vel recently, stop
        age = (self.get_clock().now() - self.last_cmd_time).nanoseconds * 1e-9
        if age > self.timeout_sec: tL = tR = 0
        else:
            tL = int(round(self.target_rpm_L * self.trim_left))
            tR = int(round(self.target_rpm_R * self.trim_right))
        self._send_line(f"SPD:{tL} {tR}")

    # ---------- Serial worker ----------
    def _io_worker(self):
        while not self.stop_event.is_set():
            try:
                self.get_logger().info(f"Opening {self.port} @ {self.baud}…")
                ser = serial.Serial(
                    self.port, self.baud,
                    timeout=0.2, write_timeout=0.5
                    # exclusive=True  # uncomment if pyserial >=3.5 and still seeing sharing
                )
                with self.ser_lock:
                    self.ser = ser
                self.get_logger().info("Serial connected.")
                # configure Arduino once
                self._send_line(f"GAIN:{self.kp:.3f} {self.ki:.3f}")
                self._send_line(f"TRM:{int(self.trim_left*100)} {int(self.trim_right*100)}")
                if self.ff_k > 0.0: self._send_line(f"FF:{self.ff_k:.3f}")

                # read loop
                while not self.stop_event.is_set():
                    line = ser.readline()
                    if not line:
                        continue  # timeout is normal
                    try:
                        s = line.decode("utf-8", errors="ignore").strip()
                    except Exception:
                        continue
                    if s:
                        self._parse_line(s)

            except Exception as e:
                self.get_logger().warn(f"Serial error: {e}. Reopening in 2s…")
                time.sleep(2)
            finally:
                with self.ser_lock:
                    try:
                        if self.ser and self.ser.is_open:
                            self.ser.close()
                    except Exception:
                        pass
                    self.ser = None

    def _send_line(self, s: str):
        with self.ser_lock:
            ser = self.ser
            if not ser or not ser.is_open:
                return
            try:
                ser.write((s + "\n").encode("ascii"))
            except Exception as e:
                # Let worker loop handle reopen
                self.get_logger().warn(f"Write failed: {e}")

    def _parse_line(self, line: str):
        # Expected lines from Arduino firmware:
        # RPM:<L> <R>   | PWM:<L> <R>   | TCK:<L> <R>
        if line.startswith("RPM:"):
            try:
                parts = line[4:].split()
                l = float(parts[0]); r = float(parts[1])
                self.pub_rpm_L.publish(Float32(data=l)); self.pub_rpm_R.publish(Float32(data=r))
                js = JointState(); js.name = ["left_wheel","right_wheel"]
                js.velocity = [l * 2*math.pi/60.0, r * 2*math.pi/60.0]
                js.header.stamp = self.get_clock().now().to_msg()
                self.pub_js.publish(js)
            except Exception:
                pass
        elif line.startswith("PWM:"):
            try:
                parts = line[4:].split()
                self.pub_pwm_L.publish(Int16(data=int(parts[0])))
                self.pub_pwm_R.publish(Int16(data=int(parts[1])))
            except Exception:
                pass
        elif line.startswith("TCK:"):
            try:
                parts = line[4:].split()
                self.pub_tck_L.publish(Int32(data=int(parts[0])))
                self.pub_tck_R.publish(Int32(data=int(parts[1])))
            except Exception:
                pass

    # ---------- lifecycle ----------
    def destroy_node(self):
        self.stop_event.set()
        return super().destroy_node()

def main(argv=None):
    rclpy.init(args=argv)
    node = ArduinoBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
